#include <jni.h>
#include <android/log.h>
#include <SDL.h>

#define LOG_TAG "DeDECTive_JNI"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// Global FD passed from Java UsbManager
int g_hackrf_fd = -1;

extern "C" {

JNIEXPORT void JNICALL
Java_com_example_dedective_MainActivity_setUsbFd(JNIEnv *env, jobject /*this*/, jint fd) {
    LOGI("Received HackRF USB File Descriptor: %d", fd);
    g_hackrf_fd = fd;
}

// In SDL2 for Android, SDL_main replaces main().
// SDL sets up the environment and calls our actual main().
// We don't need to define JNI_OnLoad because SDL handles it.
}
