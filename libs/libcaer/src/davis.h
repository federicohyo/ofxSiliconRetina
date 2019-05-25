#ifndef LIBCAER_SRC_DAVIS_H_
#define LIBCAER_SRC_DAVIS_H_

#include "davis_common.h"
#include "usb_utils.h"

#define DAVIS_DEVICE_NAME "DAVIS"

#define DAVIS_FX2_DEVICE_PID 0x841B
#define DAVIS_FX2_REQUIRED_LOGIC_REVISION 18
#define DAVIS_FX2_REQUIRED_FIRMWARE_VERSION 4

#define DAVIS_FX3_DEVICE_PID 0x841A
#define DAVIS_FX3_REQUIRED_LOGIC_REVISION 18
#define DAVIS_FX3_REQUIRED_FIRMWARE_VERSION 5

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

struct davis_handle {
	struct davis_common_handle cHandle;
	// DAVIS USB device specific state.
	struct usb_state usbState;
	struct {
		// Debug transfer support (FX3 only).
		bool enabled;
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct davis_handle *davisHandle;

ssize_t davisFindAll(caerDeviceDiscoveryResult *discoveredDevices);
ssize_t davisFindFX2(caerDeviceDiscoveryResult *discoveredDevices);
ssize_t davisFindFX3(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle davisOpenAll(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
caerDeviceHandle davisOpenFX2(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
caerDeviceHandle davisOpenFX3(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);

bool davisClose(caerDeviceHandle cdh);

bool davisSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool davisConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool davisConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool davisDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool davisDataStop(caerDeviceHandle handle);
caerEventPacketContainer davisDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DAVIS_H_ */
