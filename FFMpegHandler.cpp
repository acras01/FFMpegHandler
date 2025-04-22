#define NOMINMAX

#include "pch.h"
#include "FFMpegHandler.h"
#include "FFMpegHandlerJNI.h"

std::atomic<bool> interrupt_flag(false);

constexpr int kMaxRetryCount = 100;
constexpr int kRetryDelayMs = 10;

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

static bool waitForDisconnect(std::atomic<bool>& closedFlag, int maxRetries = kMaxRetryCount) {
    int loops = 0;
    while (!closedFlag.load() && loops++ < maxRetries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
    }
    return closedFlag.load();
}

static bool waitForInterruptClear(int maxRetries = kMaxRetryCount) {
    int attempts = 0;
    while (interrupt_flag.load() && attempts++ < maxRetries) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
    }
    return !interrupt_flag.load();
}

static LONG WINAPI MyUnhandledExceptionFilter(EXCEPTION_POINTERS* exceptionInfo) {
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
    FrameCallback frameCallback,
    SubsCallback subsCallback,
    KlvCallback klvCallback
) {
    std::string result = "";

    try {
        SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);

        if (!waitForDisconnect(isClosed)) {
            disconnect();
            closeConnection();
            return "Couldn't connect - previous connection not closed";
        }

        interrupt_flag.store(false, std::memory_order_release);

        this->sourceUrl = sourceUrl;

        for (const auto& pair : optionsMap) {
            const char* key = pair.first.c_str();
            const char* value = pair.second.c_str();

            av_dict_set(&options, key, value, 0);
            std::cout << "Setting option " << key << std::endl;
        }

        if (!waitForInterruptClear()) {
            disconnect();
            closeConnection();
            return "Couldn't open video source - input is still busy";
        }

        std::cout << "Connecting..." << std::endl;

        result = openInput();
        if (!result.empty()) return result;

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

        if (!recordFilePath.empty() && videoStreamIndex > -1) {
            this->recordFilePath = recordFilePath;
            result = setupRecordOutput(width, height, sourceFrameRate);
            if (!result.empty()) std::cout << result << std::endl;
            else std::cout << "Recording configured" << std::endl;
        }

        connectionCallback();

        isClosed.store(false, std::memory_order_release);
        isConnected = true;

        std::cout << "Start frame processing..." << std::endl;

        result = processFrames(frameCallback, subsCallback, klvCallback, width, height);

    } catch (const std::exception& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
        result = e.what();
    } catch (...) {
        std::cout << "Unknown error occurred" << std::endl;
        result = "Unknown error occurred";
    }

    disconnect();
    closeConnection();

    return result;
}

std::string FFMpegHandler::openInput() {
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
    int ret = -1;

    AVIOInterruptCB int_cb = { interrupt_callback, nullptr };
    avFormatCtx.ctx = avformat_alloc_context();
    if (!avFormatCtx) {
        return "Couldn't create input AVFormatContext";
    }
    avFormatCtx.ctx->interrupt_callback = int_cb;

    ret = avformat_open_input(&avFormatCtx, sourceUrl, nullptr, &options);
    if (ret < 0) {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        return std::string("Couldn't open video source - ") + errBuf;
    }

    ret = avformat_find_stream_info(avFormatCtx, nullptr);
    if (ret < 0) {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        return std::string("Couldn't find stream information - ") + errBuf;
    }

    videoStreamIndex = findVideoStreamIndex();
    if (videoStreamIndex < 0) {
        return "Couldn't find valid video stream inside file";
    }

    klvStreamIndex = findKlvStreamIndex();

    return "";
}

std::string FFMpegHandler::configureDecoder(const int width, const int height) {
    auto* stream = avFormatCtx.ctx->streams[videoStreamIndex];
    AVCodecParameters* avCodecParams = stream->codecpar;
    avCodecId = avCodecParams->codec_id;

    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };

    const AVCodec* avCodec = avcodec_find_decoder(avCodecId);
    if (!avCodec) {
        return "Couldn't find decoder";
    }

    // Select supported hardware config
    for (int i = 0;; ++i) {
        const AVCodecHWConfig* config = avcodec_get_hw_config(avCodec, i);
        if (!config) return "Decoder does not support defined h/w device type";

        if ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) &&
            config->device_type == hwDeviceType) {
            hwPixFmt = config->pix_fmt;
            break;
        }
    }

    avCodecCtx.ctx = avcodec_alloc_context3(avCodec);
    if (!avCodecCtx.ctx) return "Couldn't create AVCodecContext";

    if (avcodec_parameters_to_context(avCodecCtx, avCodecParams) < 0) {
        return "Couldn't initialize AVCodecContext";
    }

    avCodecCtx.ctx->thread_count = std::max(1u, std::thread::hardware_concurrency() / 2);
    avCodecCtx.ctx->flags2 |= AV_CODEC_FLAG2_FAST;
    avCodecCtx.ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;

    globalGetFormatLambda = [this](AVCodecContext*, const enum AVPixelFormat* pix_fmts) {
        for (const enum AVPixelFormat* p = pix_fmts; *p != -1; ++p) {
            if (*p == this->hwPixFmt)
                return *p;
        }
        std::cout << "Failed to get HW surface format" << std::endl;
        return AV_PIX_FMT_NONE;
        };
    avCodecCtx.ctx->get_format = getFormatWrapper;

    if (av_hwdevice_ctx_create(&hwDeviceCtx.ref, hwDeviceType, nullptr, nullptr, 0) < 0) {
        return "Failed to create specified HW device";
    }

    avCodecCtx.ctx->hw_device_ctx = av_buffer_ref(hwDeviceCtx);

    if (avcodec_open2(avCodecCtx, avCodec, nullptr) < 0) {
        return "Couldn't open codec";
    }

    swsContext.ctx = sws_getContext(
        width, height, inputHwFromat,
        width, height, outputFormat,
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );
    if (!swsContext.ctx) return "Couldn't initialize SwsContext";

    hwFrame.frame = av_frame_alloc();
    swFrame.frame = av_frame_alloc();
    pFrameRGB.frame = av_frame_alloc();
    if (!hwFrame.frame || !swFrame.frame || !pFrameRGB.frame) {
        return "Couldn't allocate AVFrame";
    }

    pFrameRGB.frame->width = width;
    pFrameRGB.frame->height = height;
    pFrameRGB.frame->format = outputFormat;

    if (av_frame_get_buffer(pFrameRGB, 32) < 0) {
        return "Failed to allocate RGB frame buffer";
    }

    bufferSize = av_image_get_buffer_size(outputFormat, width, height, 1);
    buffer = (uint8_t*)av_malloc(bufferSize);
    if (!buffer) return "Failed to allocate buffer";

    av_image_fill_arrays(pFrameRGB.frame->data, pFrameRGB.frame->linesize, buffer, outputFormat, width, height, 1);

    avPacket.packet = av_packet_alloc();
    if (!avPacket.packet) return "Couldn't allocate AVPacket";

    return "";
}

int FFMpegHandler::findVideoStreamIndex() const {
    for (unsigned int i = 0; i < avFormatCtx.ctx->nb_streams; i++) {
        if (avFormatCtx.ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            return i;
        }
    }

    return -1;
}

int FFMpegHandler::findKlvStreamIndex() const {
    for (unsigned int i = 0; i < avFormatCtx.ctx->nb_streams; i++) {
        if (avFormatCtx.ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
            return i;
        }
    }

    return -1;
}

std::string FFMpegHandler::setupUdpOutput() {
    const char* FORMAT = "mpegts";
    const char* destination = outputUrl.c_str();

    avformat_alloc_output_context2(&avOutputCtxUdp, nullptr, FORMAT, destination);
    if (!avOutputCtxUdp) {
        return "Failed to allocate AVFormatContext for udp";
    }

    for (unsigned int i = 0; i < avFormatCtx.ctx->nb_streams; ++i) {
        AVStream* inStream = avFormatCtx.ctx->streams[i];
        AVStream* outStream = avformat_new_stream(avOutputCtxUdp, nullptr);
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

    if (avformat_write_header(avOutputCtxUdp, nullptr) < 0) {
        return "Failed to write output header for udp";
    }

    isUdpOutputSet = true;

    return "";
}

std::string FFMpegHandler::setupRecordOutput(int width, int height, int sourceFrameRate) {
    frameRate = av_make_q(sourceFrameRate, 1);
    AVPixelFormat pixelFormat = AV_PIX_FMT_YUV420P;

    std::string extension = recordFilePath.substr(recordFilePath.find_last_of(".") + 1);
    const char* FORMAT;

    if (extension == "ts") {
        FORMAT = "mpegts";
    }
    else if (extension == "mkv") {
        FORMAT = "matroska";
    }
    else {
        return "Unsupported file format: " + extension;
    }

    const char* destination = recordFilePath.c_str();

    if (avformat_alloc_output_context2(&avOutputCtxRec, nullptr, FORMAT, destination) < 0) {
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

    int max_threads = std::thread::hardware_concurrency();
    encoderContextRec->thread_count = std::min<int>(max_threads, 8);
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
        av_opt_set_int(encoderContextRec->priv_data, "g", 30, 0);
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

    if (avcodec_open2(encoderContextRec, codec, nullptr) < 0) {
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

    if (FORMAT == "mpegts" && klvStreamIndex > 0) {
        AVStream* klvStream = avformat_new_stream(avOutputCtxRec, nullptr);
        if (klvStream) {
            klvStream->id = avOutputCtxRec->nb_streams - 1;
            klvStream->codecpar->codec_type = AVMEDIA_TYPE_DATA;
            klvStream->codecpar->codec_id = AV_CODEC_ID_SMPTE_KLV;
            klvStream->time_base = AVRational(1, 1000);
            klvStream->codecpar->codec_tag = MKTAG('K', 'L', 'V', ' ');
            klvStreamIndexRec = klvStream->index;
        }
        else {
            std::cerr << "Failed to allocate KLV data stream for recording" << std::endl;
        }
    }

    if (avio_open(&avOutputCtxRec->pb, destination, AVIO_FLAG_WRITE) < 0) {
        return "Couldn't open output context for recording";
    }

    if (avformat_write_header(avOutputCtxRec, nullptr) < 0) {
        return "Failed to write output header for recording";
    }

    isRecOutputSet = true;

    return "";
}

std::string FFMpegHandler::processFrames(FrameCallback callback, SubsCallback subsCallback, KlvCallback klvCallback, const int width, const int height) {
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };

    while (isConnected) {
        int ret = av_read_frame(avFormatCtx, avPacket);
        if (ret < 0) {
            av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
            std::cerr << "Failed to read frame from source: " << errBuf << std::endl;
            continue;
        }
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

        if (avPacket.packet->stream_index == videoStreamIndex) {
            ProcessResult pResult = processVideoFrame(subsCallback, width, height);
            if (pResult.message.empty() && pResult.buffer) {
                callback(pResult.buffer, av_image_get_buffer_size(outputFormat, width, height, 1));
            }
            else {
                std::cerr << "Error processing frame: " << pResult.message << std::endl;
            }
        }
        else if (avPacket.packet->stream_index == klvStreamIndex) {
            uint8_t* klvData = avPacket.packet->data;
            int klvSize = avPacket.packet->size;
            if (klvData && klvSize > 0)
                klvCallback(klvData, klvSize);

            processRecDataOutput(avPacket);
        }

        av_packet_unref(avPacket);
    }

    return "";
}

ProcessResult FFMpegHandler::processVideoFrame(SubsCallback subsCallback, const int width, const int height) {
    char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };

    int ret = avcodec_send_packet(avCodecCtx, avPacket);
    if (ret < 0) {
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        return { std::string("Failed to send AVPacket to decoder - ") + errBuf, nullptr };
    }

    while ((ret = avcodec_receive_frame(avCodecCtx, hwFrame)) == 0) {
        AVFrame* tmpFrame;
        if (hwFrame.frame->format == hwPixFmt) {
            ret = av_hwframe_transfer_data(swFrame, hwFrame, 0);
            if (ret < 0) {
                av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                std::cerr << "Error transferring the data to system memory: " << errBuf << std::endl;

                av_frame_unref(hwFrame);
                av_frame_unref(swFrame);

                continue;
            }
            tmpFrame = swFrame;
        }
        else {
            tmpFrame = hwFrame;
        }

        ret = sws_scale(swsContext, tmpFrame->data, tmpFrame->linesize, 0, height, pFrameRGB.frame->data, pFrameRGB.frame->linesize);
        if (ret < 0) {
            av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
            av_frame_unref(hwFrame);
            av_frame_unref(swFrame);
            return { errBuf, nullptr };
        }

        memcpy(buffer, pFrameRGB.frame->data[0], bufferSize);

        if (isRecOutputSet) {
            processRecVideoOutput(subsCallback, avPacket);
        }

        return { "", buffer };
    }

    av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
    return { errBuf, nullptr };
}

void FFMpegHandler::processUdpOutput(AVPacket* packet) {
    if (!avOutputCtxUdp || !packet) return;

    int streamIndex = packet->stream_index;

    if (streamMapping.find(streamIndex) == streamMapping.end()) return;

    AVStream* inStream = avFormatCtx.ctx->streams[streamIndex];
    AVStream* outStream = avOutputCtxUdp->streams[streamMapping[streamIndex]];
    AVRounding rnd = (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX);

    packet->stream_index = streamMapping[streamIndex];
    packet->pts = av_rescale_q_rnd(packet->pts, inStream->time_base, outStream->time_base, rnd);
    packet->dts = av_rescale_q_rnd(packet->dts, inStream->time_base, outStream->time_base, rnd);
    packet->duration = av_rescale_q(packet->duration, inStream->time_base, outStream->time_base);
    packet->pos = -1;

    av_interleaved_write_frame(avOutputCtxUdp, packet);
}


void FFMpegHandler::processRecVideoOutput(SubsCallback subsCallback, AVPacket* packet) {
    if (!avOutputCtxRec || !packet || videoStreamIndexRec < 0) return;

    AVStream* outStream = avOutputCtxRec->streams[videoStreamIndexRec];

    packet->stream_index = videoStreamIndexRec;
    packet->pts = currentPtsRec;
    packet->dts = packet->pts;

    int64_t duration = static_cast<int64_t>(
        av_q2d(av_div_q(av_inv_q(outStream->time_base), frameRate)) * 1
        );
    packet->duration = duration;
    currentPtsRec += duration;

    packet->pos = -1;

    if (subsCallback) subsCallback(packet->pts);

    int ret = av_interleaved_write_frame(avOutputCtxRec, packet);
    if (ret < 0) {
        char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        std::cerr << "Error writing video frame: " << errBuf << std::endl;
    }
}

void FFMpegHandler::processRecDataOutput(AVPacket* packet) {
    if (!avOutputCtxRec || !avFormatCtx || !packet || !packet->data || packet->size <= 0) return;

    if (klvStreamIndex < 0 || klvStreamIndexRec < 0) return;

    AVStream* inStream = avFormatCtx.ctx->streams[klvStreamIndex];
    AVStream* outStream = avOutputCtxRec->streams[klvStreamIndexRec];

    packet->stream_index = klvStreamIndexRec;

    if (packet->pts != AV_NOPTS_VALUE) {
        packet->pts = av_rescale_q(packet->pts, inStream->time_base, outStream->time_base);
        packet->pts = std::max(packet->pts, lastPTS + 1);
    }

    if (packet->dts != AV_NOPTS_VALUE) {
        packet->dts = av_rescale_q(packet->dts, inStream->time_base, outStream->time_base);
        packet->dts = std::max(packet->dts, lastDTS + 1);
    }

    lastPTS = packet->pts;
    lastDTS = packet->dts;

    packet->pos = -1;

    int ret = av_interleaved_write_frame(avOutputCtxRec, packet);
    if (ret < 0) {
        char errBuf[AV_ERROR_MAX_STRING_SIZE] = { 0 };
        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
        std::cerr << "Error writing data frame: " << errBuf << std::endl;
    }
}

void FFMpegHandler::disconnect() {
    std::cout << "Disconnecting" << std::endl;

    interrupt_flag.store(true, std::memory_order_release);
    isConnected = false;
}

void FFMpegHandler::closeConnection() {
    if (isClosing || isClosed) return;

    isClosing = true;
    std::cout << "Freeing resources" << std::endl;

    interrupt_flag.store(true);

    if (isRecOutputSet && avOutputCtxRec) {
        av_write_trailer(avOutputCtxRec);
        if (avOutputCtxRec->pb)
            avio_closep(&avOutputCtxRec->pb);
        avformat_free_context(avOutputCtxRec);
        avOutputCtxRec = nullptr;
        isRecOutputSet = false;
    }

    if (encoderContextRec) {
        avcodec_free_context(&encoderContextRec);
        encoderContextRec = nullptr;
    }

    if (isUdpOutputSet && avOutputCtxUdp) {
        av_write_trailer(avOutputCtxUdp);
        if (avOutputCtxUdp->pb)
            avio_closep(&avOutputCtxUdp->pb);
        avformat_free_context(avOutputCtxUdp);
        avOutputCtxUdp = nullptr;
        isUdpOutputSet = false;
    }

    // buffer is manually allocated
    if (buffer) {
        av_free(buffer);
        buffer = nullptr;
    }

    lastDTS = AV_NOPTS_VALUE;
    lastPTS = AV_NOPTS_VALUE;

    std::cout << "Resources freed" << std::endl;

    isClosed = true;
    isClosing = false;
    interrupt_flag.store(false);
}

