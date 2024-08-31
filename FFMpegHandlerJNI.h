#ifndef FFMPEGHANDLERJNI_H
#define FFMPEGHANDLERJNI_H

#pragma once

#include <jni.h>

#include <functional>

using LogCallback = std::function<void(int, const char*)>;
static LogCallback g_logCallback = nullptr;
static jobject globalCallbackRef = nullptr;

extern "C" {
    JNIEXPORT jlong JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_createHandler(JNIEnv* env, jobject obj);

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_destroyHandler(JNIEnv* env, jobject obj, jlong handlerPtr);

    JNIEXPORT jstring JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_connect(
        JNIEnv* env,
        jobject obj,
        jlong handlerPtr,
        jstring jSourceUrl,
        jstring jOutputUrl,
        jstring jRecordFilePath,
        jint width,
        jint height,
        jint sourceFrameRate,
        jobject optionsMap,
        jobject connectionCallback,
        jobject frameCallback
    );

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_disconnect(JNIEnv* env, jobject obj, jlong handlerPtr);

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_setLogCallback(JNIEnv* env, jobject obj, jobject callback);
}

#endif // FFMPEGHANDLERJNI_H