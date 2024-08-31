#include "pch.h"
#include "FFMpegHandlerJNI.h"
#include "FFMpegHandler.h"

extern "C" {

    JNIEXPORT jlong JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_createHandler(JNIEnv* env, jobject obj) {
        return (jlong)new FFMpegHandler();
    }

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_destroyHandler(JNIEnv* env, jobject obj, jlong handlerPtr) {
        try {
            if (handlerPtr != 0) {
                FFMpegHandler* handler = reinterpret_cast<FFMpegHandler*>(handlerPtr);
                delete handler;
            }
        }
        catch (const std::exception& e) {
            const char* errorMessage = e.what();
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, errorMessage);
        }
        catch (...) {
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, "Unknown error occurred during destroying");
        }
    }

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
    ) {
        try {
            FFMpegHandler* handler = reinterpret_cast<FFMpegHandler*>(handlerPtr);
            const char* sourceUrl = env->GetStringUTFChars(jSourceUrl, nullptr);
            const char* outputUrl = env->GetStringUTFChars(jOutputUrl, nullptr);
            const char* recordFilePath = env->GetStringUTFChars(jRecordFilePath, nullptr);

            std::map<std::string, std::string> options;
            if (optionsMap != nullptr) {
                jclass mapClass = env->FindClass("java/util/Map");
                jmethodID entrySetMethod = env->GetMethodID(mapClass, "entrySet", "()Ljava/util/Set;");

                jobject entrySet = env->CallObjectMethod(optionsMap, entrySetMethod);

                jclass setClass = env->FindClass("java/util/Set");
                jmethodID iteratorMethod = env->GetMethodID(setClass, "iterator", "()Ljava/util/Iterator;");

                jobject iterator = env->CallObjectMethod(entrySet, iteratorMethod);

                jclass iteratorClass = env->FindClass("java/util/Iterator");
                jmethodID hasNextMethod = env->GetMethodID(iteratorClass, "hasNext", "()Z");
                jmethodID nextMethod = env->GetMethodID(iteratorClass, "next", "()Ljava/lang/Object;");

                jclass entryClass = env->FindClass("java/util/Map$Entry");
                jmethodID getKeyMethod = env->GetMethodID(entryClass, "getKey", "()Ljava/lang/Object;");
                jmethodID getValueMethod = env->GetMethodID(entryClass, "getValue", "()Ljava/lang/Object;");

                while (env->CallBooleanMethod(iterator, hasNextMethod)) {
                    jobject entry = env->CallObjectMethod(iterator, nextMethod);

                    jstring key = (jstring)env->CallObjectMethod(entry, getKeyMethod);
                    jstring value = (jstring)env->CallObjectMethod(entry, getValueMethod);

                    const char* keyCStr = env->GetStringUTFChars(key, nullptr);
                    const char* valueCStr = env->GetStringUTFChars(value, nullptr);

                    options[keyCStr] = valueCStr;

                    env->ReleaseStringUTFChars(key, keyCStr);
                    env->ReleaseStringUTFChars(value, valueCStr);
                }
            }

            FFMpegHandler::FrameCallback frmCallback = [env, frameCallback](const uint8_t* buffer, int size) {
                jbyteArray frameData = env->NewByteArray(size);
                env->SetByteArrayRegion(frameData, 0, size, reinterpret_cast<const jbyte*>(buffer));

                jclass frameCallbackClass = env->GetObjectClass(frameCallback);
                jmethodID onFrameMethod = env->GetMethodID(frameCallbackClass, "onFrame", "([B)V");
                env->CallVoidMethod(frameCallback, onFrameMethod, frameData);

                env->DeleteLocalRef(frameData);
                };

            FFMpegHandler::ConnectionCallback conCallback = [env, connectionCallback]() {
                jclass connectionCallbackClass = env->GetObjectClass(connectionCallback);
                jmethodID onConnectedMethod = env->GetMethodID(connectionCallbackClass, "onConnected", "()V");
                env->CallVoidMethod(connectionCallback, onConnectedMethod);
                };

            auto ret = handler->connect(sourceUrl, outputUrl, recordFilePath, width, height, sourceFrameRate, options, conCallback, frmCallback);

            std::cout << "Connection result: " << ret << std::endl;

            env->ReleaseStringUTFChars(jSourceUrl, sourceUrl);
            env->ReleaseStringUTFChars(jRecordFilePath, recordFilePath);
            env->ReleaseStringUTFChars(jOutputUrl, outputUrl);

            std::cout << "Processing stopped" << std::endl;

            return env->NewStringUTF(ret.c_str());
        }
        catch (const std::exception& e) {
            const char* errorMessage = e.what();
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, errorMessage);
        }
        catch (...) {
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, "Unknown error occurred during connect");
        }
    }

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_disconnect(JNIEnv* env, jobject obj, jlong handlerPtr) {
        try {
            FFMpegHandler* handler = reinterpret_cast<FFMpegHandler*>(handlerPtr);

            if (handler != nullptr) {
                handler->disconnect();
            }
        }
        catch (const std::exception& e) {
            const char* errorMessage = e.what();
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, errorMessage);
        }
        catch (...) {
            jclass exceptionClass = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(exceptionClass, "Unknown error occurred during disconnect");
        }
    }

    JNIEXPORT void JNICALL Java_aero_swarmly_gcs_library_streamer_video_ffmpeg_FFMpegHandler_setLogCallback(JNIEnv* env, jobject obj, jobject callback) {
        if (callback == nullptr) {
            g_logCallback = nullptr;
            av_log_set_callback(nullptr);

            if (globalCallbackRef) {
                env->DeleteGlobalRef(globalCallbackRef);
                globalCallbackRef = nullptr;
            }

            std::cout << "Log callback unset" << std::endl;

            return;
        }

        if (globalCallbackRef) {
            env->DeleteGlobalRef(globalCallbackRef);
        }
        globalCallbackRef = env->NewGlobalRef(callback);

        g_logCallback = [env](int level, const char* message) {
            if (globalCallbackRef) {
                jclass callbackClass = env->GetObjectClass(globalCallbackRef);
                jmethodID callbackMethod = env->GetMethodID(callbackClass, "onLogMessage", "(ILjava/lang/String;)V");

                jstring jMessage = env->NewStringUTF(message);
                env->CallVoidMethod(globalCallbackRef, callbackMethod, level, jMessage);
                env->DeleteLocalRef(jMessage);
            }
            };

        av_log_set_callback(ffmpeg_log_callback);

        std::cout << "Log callback set" << std::endl;
    }
}
