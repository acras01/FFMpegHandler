#ifndef FFMPEGHANDLER_H
#define FFMPEGHANDLER_H

#include <map>
#include <vector>
#include <chrono>
#include <memory>
#include <iostream>
#include <thread>
#include <string>
#include <functional>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

struct ProcessResult {
    std::string message;
    uint8_t* buffer;
};

void ffmpeg_log_callback(void* ptr, int level, const char* fmt, va_list vl);
int interrupt_callback(void* ctx);
static std::function<AVPixelFormat(AVCodecContext*, const enum AVPixelFormat*)> globalGetFormatLambda;

class FFMpegHandler {
public:

    FFMpegHandler();
    ~FFMpegHandler();

    using FrameCallback = std::function<void(const uint8_t* buffer, int size)>;
    using SubsCallback = std::function<void(long time)>;
    using ConnectionCallback = std::function<void()>;

    static AVPixelFormat getFormatWrapper(AVCodecContext* ctx, const enum AVPixelFormat* pix_fmts) {
        if (globalGetFormatLambda) {
            return globalGetFormatLambda(ctx, pix_fmts);
        }
        std::cerr << "No lambda function defined" << std::endl;
        return AV_PIX_FMT_NONE;
    }

    std::string connect(
        const char* sourceUrl,
        const std::string& outputUrl,
        const std::string& recordFilePath,
        int width,
        int height,
        int sourceFrameRate,
        std::map<std::string, std::string> optionsMap,
        ConnectionCallback connectionCallback,
        FrameCallback frameCallback,
        SubsCallback subsCallback
    );
    void disconnect();

private:
    bool isConnected = false;
    bool isUdpOutputSet = false;
    bool isRecOutputSet = false;
    bool isClosing = false;
    bool isClosed = true;
    const char* sourceUrl;
    std::string subSourceUrl;
    std::string outputUrl;
    std::string recordFilePath;
    AVCodecID avCodecId = AV_CODEC_ID_NONE;
    AVDictionary* options = nullptr;
    AVFormatContext* avFormatCtx = nullptr;
    AVFrame* swFrame = nullptr;
    AVFrame* pFrameRGB = nullptr;
    AVPacket* avPacket = nullptr;
    AVCodecContext* avCodecCtx = nullptr;
    SwsContext* swsContext = nullptr;
    int bufferSize = 0;
    uint8_t* buffer = nullptr;
    int videoStreamIndex = -1;
    const AVPixelFormat outputFormat = AV_PIX_FMT_BGR0;
    const AVPixelFormat inputFormat = AV_PIX_FMT_YUV420P;

    //hw decoding
    const AVHWDeviceType hwDeviceType = AV_HWDEVICE_TYPE_D3D11VA;
    AVBufferRef* hwDeviceCtx = nullptr;
    AVFrame* hwFrame = nullptr;
    AVPixelFormat hwPixFmt = AV_PIX_FMT_NONE;
    AVPixelFormat inputHwFromat = AV_PIX_FMT_NV12;

    AVFormatContext* avOutputCtxUdp = nullptr;
    std::map<int, int> streamMapping;

    AVRational frameRate = AVRational{ 25, 1 };
    AVFormatContext* avOutputCtxRec = nullptr;
    AVCodecContext* encoderContextRec = nullptr;
    int64_t currentPtsRec = 0;
    int videoStreamIndexRec = -1;

    std::string openInput();
    std::string configureDecoder(int width, int height);
    std::string setupUdpOutput();
    void processUdpOutput(AVPacket* packet);
    std::string setupRecordOutput(int width, int height, int sourceFrameRate);
    void processRecOutput(SubsCallback subsCallback, AVPacket* packet, AVFrame* frame);
    std::string processFrameLoop(FrameCallback callback, SubsCallback subsCallback, int width, int height);
    ProcessResult processFrames(SubsCallback subsCallback, int width, int height);
    ProcessResult processVideoFrame(SubsCallback subsCallback, int width, int height);
    int findVideoStreamIndex();
    void closeConnection();
};

#endif // FFMPEGHANDLER_H
