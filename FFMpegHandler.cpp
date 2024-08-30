#include "pch.h"
#include "FFMpegHandler.h"

std::atomic<bool> interrupt_flag(false);

int interrupt_callback(void* ctx) {
    if (interrupt_flag.load()) {
        return 1;
    }
    return 0;
}

void ffmpeg_log_callback(void* ptr, int level, const char* fmt, va_list vl) {
    if (g_logCallback) {
        char messageBuffer[1024];
        vsnprintf(messageBuffer, sizeof(messageBuffer), fmt, vl);
        g_logCallback(level, messageBuffer);
    }
}

LONG WINAPI MyUnhandledExceptionFilter(EXCEPTION_POINTERS* exceptionInfo) {
    if (exceptionInfo->ExceptionRecord->ExceptionCode == EXCEPTION_ACCESS_VIOLATION) {
        std::cerr << "Access violation occurred at address: "
            << exceptionInfo->ExceptionRecord->ExceptionAddress << std::endl;

        return EXCEPTION_EXECUTE_HANDLER;
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

// Constructor
FFMpegHandler::FFMpegHandler() {
    avformat_network_init();
    av_log_set_level(AV_LOG_DEBUG);
}

// Destructor
FFMpegHandler::~FFMpegHandler() {
    closeConnection();
}

std::string FFMpegHandler::connect(
    const char* sourceUrl,
    const std::string& outputUrl,
    const std::string& recordFilePath,
    int width,
    int height,
    int sourceFrameRate,
    std::map<std::string, std::string> optionsMap,
    ConnectionCallback connectionCallback,
    FrameCallback frameCallback
) {
    try {
        SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);

        this->sourceUrl = sourceUrl;

        for (const auto& pair : optionsMap) {
            const char* key = pair.first.c_str();
            const char* value = pair.second.c_str();

            av_dict_set(&options, key, value, 0);

            std::cout << "Setting option " << key << std::endl;
        }

        std::cout << "Connecting..." << std::endl;

        auto result = openInput();
        if (!result.empty()) {
            return result;
        }
        
        std::cout << "Connection established" << std::endl;

        result = configureDecoder(width, height);
        if (!result.empty()) return result;
        std::cout << "Decoder configured" << std::endl;

        if (!outputUrl.empty()) {
            this->outputUrl = outputUrl;
            result = setupUdpOutput();
            if (!result.empty()) std::cout << result << std::endl;
            std::cout << "UDP output configured" << std::endl;
        }

        if (!recordFilePath.empty()) {
            this->recordFilePath = recordFilePath;
            result = setupRecordOutput(width, height, sourceFrameRate);
            if (!result.empty()) std::cout << result << std::endl;
            else std::cout << "Recording configured" << std::endl;

            //result = setupSubInput();
            //if (!result.empty()) std::cout << result << std::endl;
            //else std::cout << "Sub input configured" << std::endl;
        }

        connectionCallback();

        isClosed = false;
        isConnected = true;

        std::cout << "Start frame processing..." << std::endl;

        result = processFrameLoop(frameCallback, width, height);

        closeConnection();

        return result;
    }
    catch (const std::exception& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;

        return e.what();
    }
    catch (...) {
        std::cout << "Unknown error occurred" << std::endl;

        return "Unknown error occurred";
    }
}

std::string FFMpegHandler::openInput() {
    AVIOInterruptCB int_cb = { interrupt_callback, nullptr };
    avFormatCtx = avformat_alloc_context();
    if (!avFormatCtx) {
        return "Couldn't create input AVFormatContext";
    }
    avFormatCtx->interrupt_callback = int_cb;

    int ret = avformat_open_input(&avFormatCtx, sourceUrl, NULL, &options);
    if (ret != 0) {
        if (ret == AVERROR_EXIT) {
            interrupt_flag.store(false);

            std::cout << "Video source opening was interrupted" << std::endl;
            return "";
        }
        else {
            return "Couldn't open video source";
        }
    }

    ret = avformat_find_stream_info(avFormatCtx, NULL);
    if (ret < 0) {
        if (ret == AVERROR_EXIT) {
            interrupt_flag.store(false);

            std::cout << "Video source opening was interrupted" << std::endl;
            return "";
        }
        else {
            return "Couldn't find stream information";
        }
    }

    videoStreamIndex = findVideoStreamIndex();
    if (videoStreamIndex < 0) {
        return "Couldn't find valid video stream inside file";
    }

    return "";
}

std::string FFMpegHandler::configureDecoder(const int width, const int height) {
    AVCodecParameters* avCodecParams = avFormatCtx->streams[videoStreamIndex]->codecpar;
    avCodecId = avCodecParams->codec_id;
    const AVCodec* avCodec = avcodec_find_decoder(avCodecId);
    if (!avCodec) {
        return "Couldn't find decoder";
    }

    avCodecCtx = avcodec_alloc_context3(avCodec);
    if (!avCodecCtx) {
        return "Couldn't create AVCodecContext";
    }

    if (avcodec_parameters_to_context(avCodecCtx, avCodecParams) < 0) {
        return "Couldn't initialize AVCodecContext";
    }

    avCodecCtx->thread_count = max(1, std::thread::hardware_concurrency() / 2);
    avCodecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
    avCodecCtx->flags |= AV_CODEC_FLAG_LOW_DELAY;

    if (avcodec_open2(avCodecCtx, avCodec, NULL) < 0) {
        return "Couldn't open codec";
    }

    swFrame = av_frame_alloc();
    pFrameRGB = av_frame_alloc();
    if (!swFrame || !pFrameRGB) {
        return "Couldn't allocate AVFrame";
    }

    pFrameRGB->width = width;
    pFrameRGB->height = height;
    pFrameRGB->format = outputFormat;
    if (av_frame_get_buffer(pFrameRGB, 32) < 0) {
        return "Failed to allocate RGB frame buffer";
    }

    int numBytes = av_image_get_buffer_size(outputFormat, width, height, 1);
    buffer = (uint8_t*)av_malloc(numBytes);
    av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer, outputFormat, width, height, 1);

    avPacket = av_packet_alloc();
    if (!avPacket) {
        return "Couldn't allocate AVPacket";
    }

    return "";
}

int FFMpegHandler::findVideoStreamIndex() {
    for (unsigned int i = 0; i < avFormatCtx->nb_streams; i++) {
        if (avFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            return i;
        }
    }

    return -1;
}

std::string FFMpegHandler::setupUdpOutput() {
    const char* FORMAT = "mpegts";
    const char* destination = outputUrl.c_str();

    avformat_alloc_output_context2(&avOutputCtxUdp, NULL, FORMAT, destination);
    if (!avOutputCtxUdp) {
        return "Failed to allocate AVFormatContext for udp";
    }

    for (unsigned int i = 0; i < avFormatCtx->nb_streams; ++i) {
        AVStream* inStream = avFormatCtx->streams[i];
        AVStream* outStream = avformat_new_stream(avOutputCtxUdp, NULL);
        if (!outStream) {
            return "Failed to allocate output stream for udp";
        }

        if (avcodec_parameters_copy(outStream->codecpar, inStream->codecpar) < 0) {
            return "Failed to copy codec parameters for udp";
        }

        outStream->codecpar->codec_tag = 0;
        streamMapping[i] = outStream->index;
    }

    if (avio_open(&avOutputCtxUdp->pb, destination, AVIO_FLAG_WRITE) < 0) {
        return "Couldn't open output context for udp";
    }

    if (avformat_write_header(avOutputCtxUdp, NULL) < 0) {
        return "Failed to write output header for udp";
    }

    isUdpOutputSet = true;

    return "";
}

std::string FFMpegHandler::setupRecordOutput(int width, int height, int sourceFrameRate) {
    frameRate = av_make_q(sourceFrameRate, 1);
    AVPixelFormat pixelFormat = AV_PIX_FMT_YUV420P;

    const char* FORMAT = "matroska";
    const char* destination = recordFilePath.c_str();

    if (avformat_alloc_output_context2(&avOutputCtxRec, NULL, FORMAT, destination) < 0) {
        return "Failed to allocate AVFormatContext for recording";
    }

    const AVCodec* codec = avcodec_find_encoder(avCodecId);
    if (!codec) {
        return "Failed to allocate AVCodec for recording";
    }

    encoderContextRec = avcodec_alloc_context3(codec);
    if (!encoderContextRec) {
        return "Could not allocate encoder context for recording";
    }

    if (avOutputCtxRec->oformat->flags & AVFMT_GLOBALHEADER) {
        encoderContextRec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    encoderContextRec->thread_count = std::thread::hardware_concurrency() + 1;
    encoderContextRec->gop_size = 25;
    encoderContextRec->framerate = frameRate;
    encoderContextRec->time_base = av_inv_q(frameRate);
    encoderContextRec->pix_fmt = pixelFormat;
    encoderContextRec->width = width;
    encoderContextRec->height = height;
    encoderContextRec->bit_rate = 500000;
    encoderContextRec->delay = 0;

    if (avCodecId == AV_CODEC_ID_H264) {
        // x264-specific settings
        av_opt_set(encoderContextRec->priv_data, "profile", "baseline", 0);
        av_opt_set(encoderContextRec->priv_data, "level", "3.1", 0);
        av_opt_set(encoderContextRec->priv_data, "preset", "superfast", 0);
        av_opt_set(encoderContextRec->priv_data, "tune", "zerolatency", 0);
        av_opt_set(encoderContextRec->priv_data, "crf", "23", 0);
        av_opt_set_int(encoderContextRec->priv_data, "bufsize", 2000, 0);
        av_opt_set_int(encoderContextRec->priv_data, "keyint", 30, 0);
        av_opt_set_int(encoderContextRec->priv_data, "g", 1, 0);
    }
    else {
        // x265-specific settings
        av_opt_set_int(encoderContextRec->priv_data, "lookahead-slices", 2, 0); // Reduced lookahead slices
        av_opt_set_int(encoderContextRec->priv_data, "bframes", 3, 0); // Slightly reduce the number of B-frames
        av_opt_set_int(encoderContextRec->priv_data, "b-adapt", 1, 0); // Less aggressive B-frame adaptation
        av_opt_set_int(encoderContextRec->priv_data, "b-pyramid", 0, 0); // Disable B-pyramid to reduce file size
        av_opt_set(encoderContextRec->priv_data, "crf", "28", 0); // Increase CRF for lower quality but smaller size
        av_opt_set(encoderContextRec->priv_data, "qcomp", "0.50", 0); // Lower qCompress to focus more on compression
        av_opt_set_int(encoderContextRec->priv_data, "max-tu-size", 32, 0); // Larger TU size for better compression
        av_opt_set_int(encoderContextRec->priv_data, "tu-intra-depth", 1, 0);
        av_opt_set_int(encoderContextRec->priv_data, "tu-inter-depth", 1, 0);
        av_opt_set_int(encoderContextRec->priv_data, "deblock", -2, 0); // Weaker deblocking for smaller size
        av_opt_set_int(encoderContextRec->priv_data, "sao", 1, 0); // SAO still helps with quality at lower sizes
        av_opt_set_int(encoderContextRec->priv_data, "aq-mode", 1, 0); // Basic AQ mode to reduce bitrate fluctuations
        av_opt_set(encoderContextRec->priv_data, "aq-strength", "0.8", 0); // Lower AQ strength to reduce file size
    }

    if (avcodec_open2(encoderContextRec, codec, NULL) < 0) {
        return "Failed to open codec for recording";
    }

    AVStream* outStream = avformat_new_stream(avOutputCtxRec, codec);
    if (!outStream) {
        return "Failed to allocate output stream for recording";
    }

    if (avcodec_parameters_from_context(outStream->codecpar, encoderContextRec) < 0) {
        return "Failed to copy encoder context to output codec parameters for recording";
    }

    outStream->id = 0;
    outStream->codecpar->codec_id = avCodecId;
    outStream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    outStream->codecpar->format = pixelFormat;
    outStream->codecpar->width = width;
    outStream->codecpar->height = height;
    outStream->codecpar->video_delay = 0;
    outStream->time_base = av_inv_q(frameRate);
    outStream->avg_frame_rate = frameRate;
    outStream->codecpar->codec_tag = 0;

    videoStreamIndexRec = outStream->index;

    if (avio_open(&avOutputCtxRec->pb, destination, AVIO_FLAG_WRITE) < 0) {
        return "Couldn't open output context for recording";
    }

    if (avformat_write_header(avOutputCtxRec, NULL) < 0) {
        return "Failed to write output header for recording";
    }

    isRecOutputSet = true;

    return "";
}

std::string FFMpegHandler::processFrameLoop(FrameCallback callback, int width, int height) {
    while (isConnected) {
        auto processResult = processFrames(width, height);

        if (processResult.message.empty() && processResult.buffer) {
            callback(processResult.buffer.get(), av_image_get_buffer_size(outputFormat, width, height, 1));
        }
    }

    return "";
}

ProcessResult FFMpegHandler::processFrames(const int width, const int height) {
    try {
        ProcessResult result = { "", NULL };

        if (av_read_frame(avFormatCtx, avPacket) >= 0) {
            if (isUdpOutputSet) {
                AVPacket* avPacketCopy = av_packet_alloc();
                if (avPacketCopy) {
                    if (av_packet_ref(avPacketCopy, avPacket) >= 0) {
                        processUdpOutput(avPacketCopy);
                    }
                    av_packet_unref(avPacketCopy);
                    av_packet_free(&avPacketCopy);
                }
            }

            if (avPacket->stream_index == videoStreamIndex) {
                result = processVideoFrame(width, height);
            }
            av_packet_unref(avPacket);

            return result;
        }
        else {
            result.message = "Can't read frame";
            return result;
        }
    }
    catch (const std::bad_alloc& e) {
        return { "Memory allocation failed", NULL };
    }
    catch (const std::exception& e) {
        return { "Exception occurred", NULL };
    }
}

ProcessResult FFMpegHandler::processVideoFrame(const int width, const int height) {
    try {
        if (avcodec_send_packet(avCodecCtx, avPacket) < 0) {
            return { "Failed to send AVPacket to decoder", NULL };
        }

        while (avcodec_receive_frame(avCodecCtx, swFrame) >= 0) {
            if (!swsContext) {
                swsContext = sws_getContext(
                    width, height, inputFormat,
                    width, height, outputFormat,
                    SWS_BILINEAR, NULL, NULL, NULL
                );
                if (!swsContext) {
                    return { "Couldn't initialize SwsContext", NULL };
                }
            }

            sws_scale(
                swsContext,
                swFrame->data, swFrame->linesize,
                0, height,
                pFrameRGB->data, pFrameRGB->linesize
            );

            int numBytes = av_image_get_buffer_size(outputFormat, width, height, 1);
            std::unique_ptr<uint8_t[]> buffer(new uint8_t[numBytes]);

            memcpy(buffer.get(), pFrameRGB->data[0], numBytes);

            if (isRecOutputSet) {
                processRecOutput(avPacket, swFrame);
            }

            return { "", std::move(buffer) };
        }

        return { "", NULL };
    }
    catch (const std::bad_alloc& e) {
        return { "Memory allocation failed", NULL };
    }
    catch (const std::exception& e) {
        return { "Exception occurred", NULL };
    }
}

void FFMpegHandler::processUdpOutput(AVPacket* packet) {
    if (avOutputCtxUdp) {
        int streamIndex = packet->stream_index;
        AVStream* inStream = avFormatCtx->streams[streamIndex];
        AVStream* outStream = avOutputCtxUdp->streams[streamMapping.at(streamIndex)];
        AVRounding rnd = (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX);

        packet->stream_index = streamMapping.at(streamIndex);
        packet->pts = av_rescale_q_rnd(packet->pts, inStream->time_base, outStream->time_base, rnd);
        packet->dts = av_rescale_q_rnd(packet->dts, inStream->time_base, outStream->time_base, rnd);
        packet->duration = av_rescale_q(packet->duration, inStream->time_base, outStream->time_base);
        packet->pos = -1;

        av_interleaved_write_frame(avOutputCtxUdp, packet);
    }
}

void FFMpegHandler::processRecOutput(AVPacket* packet, AVFrame* frame) {
    if (avOutputCtxRec) {
        AVStream* inStream = avFormatCtx->streams[avPacket->stream_index];

        if (inStream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            AVStream* outStream = avOutputCtxRec->streams[videoStreamIndexRec];

            packet->stream_index = videoStreamIndexRec;

            packet->pts = currentPtsRec;
            packet->dts = packet->pts;

            int64_t duration = static_cast<int64_t>(av_q2d(av_div_q(av_inv_q(outStream->time_base), frameRate)) * 1);
            packet->duration = duration;
            currentPtsRec += duration;

            packet->pos = -1;

            av_interleaved_write_frame(avOutputCtxRec, packet);
        }
    }
}

void FFMpegHandler::disconnect() {
    std::cout << "Disconnecting" << std::endl;

    isConnected = false;
    interrupt_flag.store(true);
}

void FFMpegHandler::closeConnection() {
    if (!isClosing && !isClosed) {
        isClosing = true;

        std::cout << "Freeing resources" << std::endl;

        if (avFormatCtx) {
            avformat_close_input(&avFormatCtx);
            avFormatCtx = nullptr;
        }

        if (isRecOutputSet && avOutputCtxRec) {
            av_write_trailer(avOutputCtxRec);
            if (avOutputCtxRec->pb)
                avio_close(avOutputCtxRec->pb);
            avformat_free_context(avOutputCtxRec);
            if (encoderContextRec)
                avcodec_free_context(&encoderContextRec);
        }

        if (isUdpOutputSet && avOutputCtxUdp) {
            av_write_trailer(avOutputCtxUdp);
            if (avOutputCtxUdp->pb)
                avio_close(avOutputCtxUdp->pb);
            avformat_free_context(avOutputCtxUdp);
        }

        if (options) {
            av_dict_free(&options);
            options = nullptr;
        }

        if (avPacket) {
            av_packet_free(&avPacket);
            avPacket = nullptr;
        }

        if (pFrameRGB) {
            av_frame_free(&pFrameRGB);
            pFrameRGB = nullptr;
        }

        if (swFrame) {
            av_frame_free(&swFrame);
            swFrame = nullptr;
        }

        if (buffer) {
            av_free(buffer);
            buffer = nullptr;
        }

        if (swsContext) {
            sws_freeContext(swsContext);
            swsContext = nullptr;
        }

        if (avCodecCtx) {
            avcodec_free_context(&avCodecCtx);
            avCodecCtx = nullptr;
        }

        std::cout << "Resources freed" << std::endl;

        isClosed = true;
        isClosing = false;

        interrupt_flag.store(false);
    }
}
