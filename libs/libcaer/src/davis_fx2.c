#include "davis_fx2.h"

caerDeviceHandle davisFX2Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict) {
	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DAVIS_FX2_DEVICE_NAME);

	davisFX2Handle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->h.deviceType = CAER_DEVICE_DAVIS_FX2;

	bool openRetVal = davisCommonOpen((davisHandle) handle, USB_DEFAULT_DEVICE_VID, DAVIS_FX2_DEVICE_PID,
	DAVIS_FX2_DEVICE_NAME, deviceID, busNumberRestrict, devAddressRestrict, serialNumberRestrict,
	DAVIS_FX2_REQUIRED_LOGIC_REVISION, DAVIS_FX2_REQUIRED_FIRMWARE_VERSION);
	if (!openRetVal) {
		free(handle);

		// Failed to open device and grab basic information!
		return (NULL);
	}

	return ((caerDeviceHandle) handle);
}

bool davisFX2Close(caerDeviceHandle cdh) {
	davisCommonLog(CAER_LOG_DEBUG, (davisHandle) cdh, "Shutting down ...");

	return (davisCommonClose((davisHandle) cdh));
}

bool davisFX2SendDefaultConfig(caerDeviceHandle cdh) {
	// First send default chip/bias config.
	if (!davisCommonSendDefaultChipConfig(cdh, &davisFX2ConfigSet)) {
		return (false);
	}

	// Send default FPGA config.
	if (!davisCommonSendDefaultFPGAConfig(cdh, &davisFX2ConfigSet)) {
		return (false);
	}

	// FX2 specific FPGA configuration.
	if (!davisFX2ConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_ROW, 14)) {
		return (false);
	}

	if (!davisFX2ConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW, 4)) {
		return (false);
	}

	return (true);
}

bool davisFX2ConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	davisHandle handle = (davisHandle) cdh;

	if (modAddr == DAVIS_CONFIG_USB && paramAddr == DAVIS_CONFIG_USB_EARLY_PACKET_DELAY) {
		// Early packet delay is 125µs slices on host, but in cycles
		// @ USB_CLOCK_FREQ on FPGA, so we must multiply here.
		param = param * (125 * DAVIS_FX2_USB_CLOCK_FREQ);
	}

	return (davisCommonConfigSet(handle, modAddr, paramAddr, param));
}

bool davisFX2ConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	davisHandle handle = (davisHandle) cdh;

	bool retVal = davisCommonConfigGet(handle, modAddr, paramAddr, param);

	if (retVal && modAddr == DAVIS_CONFIG_USB && paramAddr == DAVIS_CONFIG_USB_EARLY_PACKET_DELAY) {
		// Early packet delay is 125µs slices on host, but in cycles
		// @ USB_CLOCK_FREQ on FPGA, so we must divide here.
		*param = *param / (125 * DAVIS_FX2_USB_CLOCK_FREQ);
	}

	return (retVal);
}
