#ifndef LIBUSB_H
#define LIBUSB_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct libusb_context libusb_context;
typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;
extern libusb_context* g_libusb_context;
int libusb_init(libusb_context **ctx);
void libusb_exit(libusb_context *ctx);
void libusb_close(libusb_device_handle *dev_handle);
int libusb_wrap_sys_device(libusb_context *ctx, intptr_t sys_dev, libusb_device_handle **dev_handle);
#define LIBUSB_SUCCESS 0
#ifdef __cplusplus
}
#endif
#endif
