/**
 * @file usb.h
 *
 * Common functions to access, configure and exchange data with
 * supported USB devices. Also contains defines for USB specific
 * configuration options.
 */

#ifndef LIBCAER_DEVICES_USB_H_
#define LIBCAER_DEVICES_USB_H_

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Module address: host-side USB configuration.
 */
#define CAER_HOST_CONFIG_USB -1

/**
 * Parameter address for module CAER_HOST_CONFIG_USB:
 * set number of buffers used by libusb for asynchronous data transfers
 * with the USB device. The default values are usually fine, only change
 * them if you're running into I/O limits.
 */
#define CAER_HOST_CONFIG_USB_BUFFER_NUMBER 0
/**
 * Parameter address for module CAER_HOST_CONFIG_USB:
 * set size of each buffer used by libusb for asynchronous data transfers
 * with the USB device. The default values are usually fine, only change
 * them if you're running into I/O limits.
 */
#define CAER_HOST_CONFIG_USB_BUFFER_SIZE 1

/**
 * Open a specified USB device, assign an ID to it and return a handle for further usage.
 * Various means can be employed to limit the selection of the device.
 *
 * @param deviceID a unique ID to identify the device from others. Will be used as the
 *                 source for EventPackets being generated from its data.
 * @param deviceType type of the device to open. Currently supported are:
 *                   CAER_DEVICE_DVS128, CAER_DEVICE_DAVIS, CAER_DEVICE_DYNAPSE
 * @param busNumberRestrict restrict the search for viable devices to only this USB bus number.
 * @param devAddressRestrict restrict the search for viable devices to only this USB device address.
 * @param serialNumberRestrict restrict the search for viable devices to only devices which do
 *                             possess the given Serial Number in their USB SerialNumber descriptor.
 *
 * @return a valid device handle that can be used with the other libcaer functions,
 *         or NULL on error. Always check for this! On error, errno is also set to
 *         provide more precise information about the failure cause.
 */
caerDeviceHandle caerDeviceOpen(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_USB_H_ */
