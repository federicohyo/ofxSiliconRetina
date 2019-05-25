/**
 * @file serial.h
 *
 * Common functions to access, configure and exchange data with
 * supported serial port devices. Also contains defines for serial
 * port specific configuration options.
 */

#ifndef LIBCAER_DEVICES_SERIAL_H_
#define LIBCAER_DEVICES_SERIAL_H_

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Module address: host-side serial port configuration.
 */
#define CAER_HOST_CONFIG_SERIAL -1

/**
 * Parameter address for module CAER_HOST_CONFIG_SERIAL:
 * read size for serial port communication.
 */
#define CAER_HOST_CONFIG_SERIAL_READ_SIZE 0

/**
 * Parameter values for module CAER_HOST_CONFIG_SERIAL:
 * possible baud-rates for serial port communication.
 */
//@{
#define CAER_HOST_CONFIG_SERIAL_BAUD_RATE_2M 2000000
#define CAER_HOST_CONFIG_SERIAL_BAUD_RATE_4M 4000000
#define CAER_HOST_CONFIG_SERIAL_BAUD_RATE_8M 8000000
#define CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M 12000000
//@}

/**
 * Open a specified serial port device, assign an ID to it and return a handle for
 * further usage. Various means can be employed to limit the selection of the device.
 *
 * @param deviceID a unique ID to identify the device from others. Will be used as the
 *                 source for EventPackets being generated from its data.
 * @param deviceType type of the device to open. Currently supported are: CAER_DEVICE_EDVS
 * @param serialPortName name of the serial port device to open.
 * @param serialBaudRate baud-rate for serial port communication.
 *
 * @return a valid device handle that can be used with the other libcaer functions,
 *         or NULL on error. Always check for this! On error, errno is also set to
 *         provide more precise information about the failure cause.
 */
caerDeviceHandle caerDeviceOpenSerial(
	uint16_t deviceID, uint16_t deviceType, const char *serialPortName, uint32_t serialBaudRate);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_SERIAL_H_ */
