/**
 * @file device_discover.h
 *
 * Functions to discover supported devices attached to the
 * current host system, and then open them.
 */

#ifndef LIBCAER_DEVICES_DEVICE_DISCOVER_H_
#define LIBCAER_DEVICES_DEVICE_DISCOVER_H_

#include "davis.h"
#include "dvs128.h"
#include "dynapse.h"
#include "edvs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Result of a device discovery operation.
 * Contains the type of the device and its informational structure; use the
 * device type to properly select the right info structure!
 * In the info structures, 'deviceID' will always be set to -1 and
 * 'deviceString' will always be NULL, as those are not present during
 * the generic discovery phase.
 */
struct caer_device_discovery_result {
	uint16_t deviceType;
	bool deviceErrorOpen;
	bool deviceErrorVersion;
	union {
		struct caer_dvs128_info dvs128Info;
		struct caer_edvs_info edvsInfo;
		struct caer_davis_info davisInfo;
		struct caer_dynapse_info dynapseInfo;
	} deviceInfo;
};

/**
 * Pointer to result of a device discovery operation.
 */
typedef struct caer_device_discovery_result *caerDeviceDiscoveryResult;

/**
 * Define for special value to discover all device types.
 */
#define CAER_DEVICE_DISCOVER_ALL -1

/**
 * Discover all supported devices that are accessible on this system.
 * Use -1 as 'deviceType' to search for any device, or an actual
 * device type ID to only search for matches of that specific type.
 *
 * @param deviceType type of device to search for, use -1 for any.
 * @param discoveredDevices pointer to array of results, memory will be
 *                          allocated for it automatically. On error,
 *                          the pointer is set to NULL. Remember to free()
 *                          the memory once done!
 *
 * @return number of discovered devices, 0 if no device could be found;
 *         or -1 if an error occurred.
 */
ssize_t caerDeviceDiscover(int16_t deviceType, caerDeviceDiscoveryResult *discoveredDevices);

/**
 * Open a specific device based on information returned by caerDeviceDiscover(),
 * then assign an ID to it and return a handle for further usage.
 *
 * @param deviceID a unique ID to identify the device from others. Will be used as the
 *                 source for EventPackets being generated from its data.
 * @param discoveredDevice pointer to the result of a device discovery operation.
 *                         Uniquely identifies a particular device.
 *
 * @return a valid device handle that can be used with the other libcaer functions,
 *         or NULL on error. Always check for this!
 */
caerDeviceHandle caerDeviceDiscoverOpen(uint16_t deviceID, caerDeviceDiscoveryResult discoveredDevice);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DEVICE_DISCOVER_H_ */
