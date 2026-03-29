#!/bin/bash
# Remove the old patch definition from the C header
sed -i 's/extern ADDAPI int ADDCALL hackrf_open_by_libusb_handle(struct libusb_device_handle\* handle, hackrf_device\*\* device);//g' vendor/libhackrf/src/hackrf.h
sed -i 's/int ADDCALL hackrf_open_by_libusb_handle(libusb_device_handle\* handle, hackrf_device\*\* device)//g' vendor/libhackrf/src/hackrf.c
