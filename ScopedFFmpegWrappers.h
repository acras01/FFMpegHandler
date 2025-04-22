// ScopedFFmpegWrappers.h
#pragma once

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

#include <memory>

struct ScopedAVFormatCtx {
    AVFormatContext* ctx = nullptr;
    ~ScopedAVFormatCtx() {
        if (ctx) avformat_close_input(&ctx);
    }
    operator AVFormatContext* () const { return ctx; }
    AVFormatContext** operator&() { return &ctx; }
};

struct ScopedAVFrame {
    AVFrame* frame = av_frame_alloc();
    ~ScopedAVFrame() {
        if (frame) av_frame_free(&frame);
    }
    operator AVFrame* () const { return frame; }
};

struct ScopedAVPacket {
    AVPacket* packet = av_packet_alloc();
    ~ScopedAVPacket() {
        if (packet) av_packet_free(&packet);
    }
    operator AVPacket* () const { return packet; }
};

struct ScopedSwsContext {
    SwsContext* ctx = nullptr;
    ~ScopedSwsContext() {
        if (ctx) sws_freeContext(ctx);
    }
    operator SwsContext* () const { return ctx; }
};

struct ScopedAVCodecCtx {
    AVCodecContext* ctx = nullptr;
    ScopedAVCodecCtx(const AVCodec* codec) {
        ctx = avcodec_alloc_context3(codec);
    }
    ~ScopedAVCodecCtx() {
        if (ctx) avcodec_free_context(&ctx);
    }
    operator AVCodecContext* () const { return ctx; }
    AVCodecContext** operator&() { return &ctx; }
};

struct ScopedAVBufferRef {
    AVBufferRef* ref = nullptr;
    ~ScopedAVBufferRef() {
        if (ref) av_buffer_unref(&ref);
    }
    operator AVBufferRef* () const { return ref; }
    AVBufferRef** operator&() { return &ref; }
};

struct ScopedAVDictionary {
    AVDictionary* dict = nullptr;
    ~ScopedAVDictionary() {
        if (dict) av_dict_free(&dict);
    }
    operator AVDictionary* () const { return dict; }
    AVDictionary** operator&() { return &dict; }
};

