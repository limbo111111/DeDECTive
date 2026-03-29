# DeDECTive Android Build Guide

Porting the DeDECTive C++ application to run natively on Android involves a few platform-specific configurations, mainly related to hardware access and optimized performance via DSP libraries. Here's exactly how to set up your Android NDK project to build and run the application flawlessly.

## 1. Hardware Access: Passing the HackRF USB File Descriptor

Android apps running without root privileges cannot directly scan the USB bus and open `/dev/bus/usb/xxx/yyy`. They must request permission from the user through the `UsbManager` in Java/Kotlin.

1.  In your `MainActivity.java` or `MainActivity.kt`, find the HackRF device (Vendor ID `0x1d50`, Product ID `0x6089`).
2.  Use `UsbManager.requestPermission()` to get user approval.
3.  Once approved, call `usbManager.openDevice(device)`. This returns a `UsbDeviceConnection`.
4.  Get the file descriptor via `usbDeviceConnection.getFileDescriptor()`.
5.  Pass this integer file descriptor down to your native C++ code using the JNI function defined in `android_jni.cpp`:
    `public native void setUsbFd(int fd);`

### Patching `libhackrf`
The standard C `libhackrf` library does not contain a function to accept an already-opened `libusb_device_handle`. Since we must use Android's `UsbManager` file descriptor, we need to inject our handle into the `hackrf_device` structure.

Instead of searching for outdated or buggy Android forks, use the **official `libhackrf` source code** (or add it as a submodule to your NDK build), and simply add the following C function to the very bottom of `src/hackrf.c`:

```c
// HackRF Android Port helper:
// Allows wrapping a pre-opened libusb_device_handle (created from a Java
// UsbDeviceConnection File Descriptor) into a fully initialized hackrf_device.
int ADDCALL hackrf_open_by_libusb_handle(libusb_device_handle* handle, hackrf_device** device)
{
	if (handle == NULL || device == NULL) {
		return HACKRF_ERROR_INVALID_PARAM;
	}

	// Delegate the heavy lifting to the internal, static initialization
	// function used by hackrf_open() and hackrf_device_list_open()
	return hackrf_open_setup(handle, device);
}
```

*Note: Because `hackrf_open_setup` is a `static` internal function inside `hackrf.c`, you **must** place this new function inside `hackrf.c`. This guarantees your Android USB File Descriptor initializes the exact same async transfer threads, interface claims, and buffers as a standard desktop Linux USB connection, eliminating any unexpected runtime failures.*

Our modified `hackrf_source.cpp` code handles wrapping the raw Java integer File Descriptor into a valid `libusb_device_handle` (via `libusb_wrap_sys_device()`) and passes it seamlessly to this new function.

## 2. DSP Dependencies: KissFFT

The raw `fft_inplace` method used on desktop targets is completely unoptimized for mobile processors and will result in terrible battery life and massive thermal throttling (stuttering). The Android build utilizes **KissFFT** for extremely fast, NEON-optimized FFT computation.

1.  Download the `kiss_fft` source code.
2.  Add it to your `CMakeLists.txt` and ensure it's compiled with `KISS_FFT_USE_ALLOCA` or standard float support. **Do not** compile it with the `-DFIXED_POINT` flag, as our code expects and relies on the exact memory layout parity between `std::complex<float>` and `kiss_fft_cpx` (which are both standard 32-bit floating point representations).

## 3. Audio Dependency: Google Oboe

PulseAudio (`libpulse`) does not exist on Android. We have completely rewritten the `AudioOutput` class to utilize **Google Oboe**, the official, low-latency native audio API for Android (it automatically routes to AAudio or OpenSL ES depending on device capabilities).

You must import Oboe via Android Studio's Prefab system:

1.  In your app-level `build.gradle` (or `build.gradle.kts`):
    ```groovy
    android {
        buildFeatures {
            prefab true
        }
    }
    dependencies {
        implementation 'com.google.oboe:oboe:1.8.1' // Or latest version
    }
    ```
2.  Your NDK `CMakeLists.txt` will automatically discover and link Oboe using `find_package(oboe REQUIRED CONFIG)`.

## 4. NDK Build Settings & Compiler Flags

To maximize performance, especially the ARM NEON optimizations implemented in the 18Msps mixer loop, configure your Gradle to target `arm64-v8a` explicitly.

```groovy
android {
    defaultConfig {
        externalNativeBuild {
            cmake {
                arguments "-DANDROID_STL=c++_shared"
                cppFlags "-O3 -flto -ffast-math"
                abiFilters "arm64-v8a"
            }
        }
    }
}
```

By following this guide, DeDECTive will leverage the full potential of native Android APIs (SDL2, OpenGL ES 3.0, Oboe, libusb wrapping, and ARM NEON intrinsics), delivering a fast, efficient, and production-ready SDR decoding experience.
