#include "davis.h"

static ssize_t davisFindInternal(uint16_t deviceType, caerDeviceDiscoveryResult *discoveredDevices);

static caerDeviceHandle davisOpenInternal(uint16_t deviceType, uint16_t deviceID, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict);

static void davisEventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent);

// FX3 Debug Transfer Support
static void allocateDebugTransfers(davisHandle handle);
static void cancelAndDeallocateDebugTransfers(davisHandle handle);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(davisHandle handle, const uint8_t *buffer, size_t bytesSent);

ssize_t davisFindAll(caerDeviceDiscoveryResult *discoveredDevices) {
	return (davisFindInternal(CAER_DEVICE_DAVIS, discoveredDevices));
}

ssize_t davisFindFX2(caerDeviceDiscoveryResult *discoveredDevices) {
	return (davisFindInternal(CAER_DEVICE_DAVIS_FX2, discoveredDevices));
}

ssize_t davisFindFX3(caerDeviceDiscoveryResult *discoveredDevices) {
	return (davisFindInternal(CAER_DEVICE_DAVIS_FX3, discoveredDevices));
}

static ssize_t davisFindInternal(uint16_t deviceType, caerDeviceDiscoveryResult *discoveredDevices) {
	// Set to NULL initially (for error return).
	*discoveredDevices = NULL;

	ssize_t resultFX2              = 0;
	struct usb_info *foundDavisFX2 = NULL;

	if ((deviceType == CAER_DEVICE_DAVIS) || (deviceType == CAER_DEVICE_DAVIS_FX2)) {
		resultFX2 = usbDeviceFind(USB_DEFAULT_DEVICE_VID, DAVIS_FX2_DEVICE_PID, DAVIS_FX2_REQUIRED_LOGIC_REVISION,
			DAVIS_FX2_REQUIRED_FIRMWARE_VERSION, &foundDavisFX2);
	}

	ssize_t resultFX3              = 0;
	struct usb_info *foundDavisFX3 = NULL;

	if ((deviceType == CAER_DEVICE_DAVIS) || (deviceType == CAER_DEVICE_DAVIS_FX3)) {
		resultFX3 = usbDeviceFind(USB_DEFAULT_DEVICE_VID, DAVIS_FX3_DEVICE_PID, DAVIS_FX3_REQUIRED_LOGIC_REVISION,
			DAVIS_FX3_REQUIRED_FIRMWARE_VERSION, &foundDavisFX3);
	}

	if ((resultFX2 < 0) || (resultFX3 < 0)) {
		// Free any possible found devices.
		free(foundDavisFX2);
		free(foundDavisFX3);

		// Error code, return right away.
		return (-1);
	}

	size_t resultAll = (size_t)(resultFX2 + resultFX3);

	if (resultAll == 0) {
		// Nothing found. No devices to free.
		return (0);
	}

	// Allocate memory for discovered devices in expected format.
	*discoveredDevices = calloc(resultAll, sizeof(struct caer_device_discovery_result));
	if (*discoveredDevices == NULL) {
		free(foundDavisFX2);
		free(foundDavisFX3);

		return (-1);
	}

	// Transform from generic USB format into device discovery one.
	size_t j = 0;

	caerLogDisable(true);
	for (size_t i = 0; i < (size_t) resultFX2; i++, j++) {
		// This is a DAVIS FX2.
		(*discoveredDevices)[j].deviceType         = CAER_DEVICE_DAVIS_FX2;
		(*discoveredDevices)[j].deviceErrorOpen    = foundDavisFX2[i].errorOpen;
		(*discoveredDevices)[j].deviceErrorVersion = foundDavisFX2[i].errorVersion;
		struct caer_davis_info *davisInfoPtr       = &((*discoveredDevices)[j].deviceInfo.davisInfo);

		davisInfoPtr->deviceUSBBusNumber     = foundDavisFX2[i].busNumber;
		davisInfoPtr->deviceUSBDeviceAddress = foundDavisFX2[i].devAddress;
		strncpy(davisInfoPtr->deviceSerialNumber, foundDavisFX2[i].serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

		// Reopen DAVIS device to get additional info, if possible at all.
		if (!foundDavisFX2[i].errorOpen && !foundDavisFX2[i].errorVersion) {
			caerDeviceHandle davis
				= davisOpenFX2(0, davisInfoPtr->deviceUSBBusNumber, davisInfoPtr->deviceUSBDeviceAddress, NULL);
			if (davis != NULL) {
				*davisInfoPtr = caerDavisInfoGet(davis);

				davisClose(davis);
			}
		}

		// Set/Reset to invalid values, not part of discovery.
		davisInfoPtr->deviceID     = -1;
		davisInfoPtr->deviceString = NULL;
	}

	for (size_t i = 0; i < (size_t) resultFX3; i++, j++) {
		// This is a DAVIS FX3.
		(*discoveredDevices)[j].deviceType         = CAER_DEVICE_DAVIS_FX3;
		(*discoveredDevices)[j].deviceErrorOpen    = foundDavisFX3[i].errorOpen;
		(*discoveredDevices)[j].deviceErrorVersion = foundDavisFX3[i].errorVersion;
		struct caer_davis_info *davisInfoPtr       = &((*discoveredDevices)[j].deviceInfo.davisInfo);

		davisInfoPtr->deviceUSBBusNumber     = foundDavisFX3[i].busNumber;
		davisInfoPtr->deviceUSBDeviceAddress = foundDavisFX3[i].devAddress;
		strncpy(davisInfoPtr->deviceSerialNumber, foundDavisFX3[i].serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

		// Reopen DAVIS device to get additional info, if possible at all.
		if (!foundDavisFX3[i].errorOpen && !foundDavisFX3[i].errorVersion) {
			caerDeviceHandle davis
				= davisOpenFX3(0, davisInfoPtr->deviceUSBBusNumber, davisInfoPtr->deviceUSBDeviceAddress, NULL);
			if (davis != NULL) {
				*davisInfoPtr = caerDavisInfoGet(davis);

				davisClose(davis);
			}
		}

		// Set/Reset to invalid values, not part of discovery.
		davisInfoPtr->deviceID     = -1;
		davisInfoPtr->deviceString = NULL;
	}
	caerLogDisable(false);

	free(foundDavisFX2);
	free(foundDavisFX3);
	return ((ssize_t) resultAll);
}

caerDeviceHandle davisOpenAll(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	return (
		davisOpenInternal(CAER_DEVICE_DAVIS, deviceID, busNumberRestrict, devAddressRestrict, serialNumberRestrict));
}

caerDeviceHandle davisOpenFX2(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	return (davisOpenInternal(
		CAER_DEVICE_DAVIS_FX2, deviceID, busNumberRestrict, devAddressRestrict, serialNumberRestrict));
}

caerDeviceHandle davisOpenFX3(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	return (davisOpenInternal(
		CAER_DEVICE_DAVIS_FX3, deviceID, busNumberRestrict, devAddressRestrict, serialNumberRestrict));
}

static caerDeviceHandle davisOpenInternal(uint16_t deviceType, uint16_t deviceID, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	errno = 0;

	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DAVIS_DEVICE_NAME);

	davisHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->cHandle.deviceType = deviceType;

	// Setup common handling.
	handle->cHandle.spiConfigPtr = &handle->usbState;

	davisCommonState state = &handle->cHandle.state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	dataExchangeSettingsInit(&state->dataExchange);

	// Packet settings (size (in events) and time interval (in µs)).
	containerGenerationSettingsInit(&state->container);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	atomic_store(&handle->usbState.usbLogLevel, globalLogLevel);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, DAVIS_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&handle->usbState, usbThreadName);
	handle->cHandle.info.deviceString = usbThreadName; // Temporary, until replaced by full string.

	// Try to open a DAVIS device on a specific USB port.
	bool deviceFound        = false;
	struct usb_info usbInfo = {0, 0, "", false, false, 0, 0};

	if ((deviceType == CAER_DEVICE_DAVIS) || (deviceType == CAER_DEVICE_DAVIS_FX2)) {
		deviceFound = usbDeviceOpen(&handle->usbState, USB_DEFAULT_DEVICE_VID, DAVIS_FX2_DEVICE_PID, busNumberRestrict,
			devAddressRestrict, serialNumberRestrict, DAVIS_FX2_REQUIRED_LOGIC_REVISION,
			DAVIS_FX2_REQUIRED_FIRMWARE_VERSION, &usbInfo);
	}

	if ((!deviceFound) && ((deviceType == CAER_DEVICE_DAVIS) || (deviceType == CAER_DEVICE_DAVIS_FX3))) {
		deviceFound = usbDeviceOpen(&handle->usbState, USB_DEFAULT_DEVICE_VID, DAVIS_FX3_DEVICE_PID, busNumberRestrict,
			devAddressRestrict, serialNumberRestrict, DAVIS_FX3_REQUIRED_LOGIC_REVISION,
			DAVIS_FX3_REQUIRED_FIRMWARE_VERSION, &usbInfo);

		if (deviceFound) {
			handle->fx3Support.enabled = true;
		}
	}

	if (!deviceFound) {
		if (errno == CAER_ERROR_OPEN_ACCESS) {
			davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
				"Failed to open device, no matching device could be found or opened.");
		}
		else {
			davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
				"Failed to open device, see above log message for more information (errno=%d).", errno);
		}

		free(handle);

		// errno set by usbDeviceOpen().
		return (NULL);
	}

	char *usbInfoString = usbGenerateDeviceString(usbInfo, DAVIS_DEVICE_NAME, deviceID);
	if (usbInfoString == NULL) {
		davisLog(CAER_LOG_CRITICAL, &handle->cHandle, "Failed to generate USB information string.");

		usbDeviceClose(&handle->usbState);
		free(handle);

		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	// Setup USB.
	usbSetDataCallback(&handle->usbState, &davisEventTranslator, handle);
	usbSetDataEndpoint(&handle->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&handle->usbState, 8);
	usbSetTransfersSize(&handle->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&handle->usbState)) {
		usbDeviceClose(&handle->usbState);
		free(usbInfoString);
		free(handle);

		errno = CAER_ERROR_COMMUNICATION;
		return (NULL);
	}

	// Populate info variables based on data from device.
	handle->cHandle.info.deviceID = I16T(deviceID);
	strncpy(handle->cHandle.info.deviceSerialNumber, usbInfo.serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);
	handle->cHandle.info.deviceUSBBusNumber     = usbInfo.busNumber;
	handle->cHandle.info.deviceUSBDeviceAddress = usbInfo.devAddress;
	handle->cHandle.info.deviceString           = usbInfoString;

	handle->cHandle.info.firmwareVersion = usbInfo.firmwareVersion;
	handle->cHandle.info.logicVersion    = usbInfo.logicVersion;

	davisCommonInit(&handle->cHandle);

	// On FX3, start the debug transfers once everything else is ready.
	if (handle->fx3Support.enabled) {
		allocateDebugTransfers(handle);
	}

	davisLog(CAER_LOG_DEBUG, &handle->cHandle,
		"Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".", usbInfo.busNumber,
		usbInfo.devAddress);

	return ((caerDeviceHandle) handle);
}

bool davisClose(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;

	davisLog(CAER_LOG_DEBUG, &handle->cHandle, "Shutting down ...");

	// Stop debug transfers on FX3 devices.
	if (handle->fx3Support.enabled) {
		cancelAndDeallocateDebugTransfers(handle);
	}

	// Shut down USB handling thread.
	usbThreadStop(&handle->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&handle->usbState);

	davisLog(CAER_LOG_DEBUG, &handle->cHandle, "Shutdown successful.");

	// Free memory.
	free(handle->cHandle.info.deviceString);
	free(handle);

	return (true);
}

bool davisSendDefaultConfig(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;

	// First send default chip/bias config.
	if (!davisCommonSendDefaultChipConfig(&handle->cHandle)) {
		return (false);
	}

	// Send default FPGA config.
	if (!davisCommonSendDefaultFPGAConfig(&handle->cHandle)) {
		return (false);
	}

	davisConfigSet(cdh, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs blocks (so 1ms)

	return (true);
}

bool davisConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	davisHandle handle = (davisHandle) cdh;

	if (modAddr == CAER_HOST_CONFIG_USB) {
		return (usbConfigSet(&handle->usbState, paramAddr, param));
	}
	else if (modAddr == CAER_HOST_CONFIG_LOG && paramAddr == CAER_HOST_CONFIG_LOG_LEVEL) {
		// Set USB log-level to this value too.
		atomic_store(&handle->usbState.usbLogLevel, U8T(param));

		// Also set standard device log-level.
		return (davisCommonConfigSet(&handle->cHandle, CAER_HOST_CONFIG_LOG, CAER_HOST_CONFIG_LOG_LEVEL, param));
	}
	else if (modAddr == DAVIS_CONFIG_USB) {
		switch (paramAddr) {
			case DAVIS_CONFIG_USB_RUN:
				return (spiConfigSend(handle->cHandle.spiConfigPtr, DAVIS_CONFIG_USB, paramAddr, param));
				break;

			case DAVIS_CONFIG_USB_EARLY_PACKET_DELAY: {
				// Early packet delay is 125µs slices on host, but in cycles
				// @ USB_CLOCK_FREQ on FPGA, so we must multiply here.
				float delayCC = roundf((float) param * 125.0F * handle->cHandle.state.deviceClocks.usbClockActual);
				return (spiConfigSend(handle->cHandle.spiConfigPtr, DAVIS_CONFIG_USB, paramAddr, U32T(delayCC)));
				break;
			}

			default:
				return (false);
				break;
		}
	}

	// Common config call.
	return (davisCommonConfigSet(&handle->cHandle, modAddr, paramAddr, param));
}

bool davisConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	davisHandle handle = (davisHandle) cdh;

	if (modAddr == CAER_HOST_CONFIG_USB) {
		return (usbConfigGet(&handle->usbState, paramAddr, param));
	}
	else if (modAddr == DAVIS_CONFIG_USB) {
		switch (paramAddr) {
			case DAVIS_CONFIG_USB_RUN:
				return (spiConfigReceive(handle->cHandle.spiConfigPtr, DAVIS_CONFIG_USB, paramAddr, param));
				break;

			case DAVIS_CONFIG_USB_EARLY_PACKET_DELAY: {
				// Early packet delay is 125µs slices on host, but in cycles
				// @ USB_CLOCK_FREQ on FPGA, so we must divide here.
				uint32_t cyclesValue = 0;
				if (!spiConfigReceive(handle->cHandle.spiConfigPtr, DAVIS_CONFIG_USB, paramAddr, &cyclesValue)) {
					return (false);
				}

				float delayCC
					= roundf((float) cyclesValue / (125.0F * handle->cHandle.state.deviceClocks.usbClockActual));
				*param = U32T(delayCC);

				return (true);
				break;
			}

			default:
				return (false);
				break;
		}
	}

	// Common config call.
	return (davisCommonConfigGet(&handle->cHandle, modAddr, paramAddr, param));
}

bool davisDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
	void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) {
	davisHandle handle     = (davisHandle) cdh;
	davisCommonState state = &handle->cHandle.state;

	usbSetShutdownCallback(&handle->usbState, dataShutdownNotify, dataShutdownUserPtr);

	if (!davisCommonDataStart(&handle->cHandle, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr)) {
		return (false);
	}

	if (!usbDataTransfersStart(&handle->usbState)) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, &handle->cHandle, "Failed to start data transfers.");
		return (false);
	}

	if (dataExchangeStartProducers(&state->dataExchange)) {
		// Enable data transfer on USB end-point 2.
		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN_CHIP, true);

		// Wait 200 ms for biases to stabilize.
		struct timespec biasEnSleep = {.tv_sec = 0, .tv_nsec = 200000000};
		thrd_sleep(&biasEnSleep, NULL);

		davisConfigSet(cdh, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, true);
		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, true);
		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, true);

		// Wait 50 ms for data transfer to be ready.
		struct timespec noDataSleep = {.tv_sec = 0, .tv_nsec = 50000000};
		thrd_sleep(&noDataSleep, NULL);

		davisConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true);
		davisConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, true);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, true);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, true);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, true);
		davisConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, true);
	}

	return (true);
}

bool davisDataStop(caerDeviceHandle cdh) {
	davisHandle handle     = (davisHandle) cdh;
	davisCommonState state = &handle->cHandle.state;

	if (dataExchangeStopProducers(&state->dataExchange)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		davisConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, false);
		davisConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, false);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, false);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, false);
		davisConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, false);
		davisConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, false);

		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, false);
		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, false);
		davisConfigSet(cdh, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, false);

		davisConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN_CHIP, false);
	}

	usbDataTransfersStop(&handle->usbState);

	davisCommonDataStop(&handle->cHandle);

	return (true);
}

caerEventPacketContainer davisDataGet(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;

	return (dataExchangeGet(&handle->cHandle.state.dataExchange, &handle->usbState.dataTransfersRun));
}

static void davisEventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent) {
	davisHandle handle = (davisHandle) vhd;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&handle->usbState)) {
		return;
	}

	davisCommonEventTranslator(&handle->cHandle, buffer, bytesSent, &handle->usbState.dataTransfersRun);
}

//////////////////////////////////
/// FX3 Debug Transfer Support ///
//////////////////////////////////
static void allocateDebugTransfers(davisHandle handle) {
	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		handle->fx3Support.debugTransfers[i] = libusb_alloc_transfer(0);
		if (handle->fx3Support.debugTransfers[i] == NULL) {
			davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
				"Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
				DEBUG_TRANSFER_NUM);
			continue;
		}

		// Create data buffer.
		handle->fx3Support.debugTransfers[i]->length = DEBUG_TRANSFER_SIZE;
		handle->fx3Support.debugTransfers[i]->buffer = malloc(DEBUG_TRANSFER_SIZE);
		if (handle->fx3Support.debugTransfers[i]->buffer == NULL) {
			davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
				"Unable to allocate buffer for libusb transfer %zu (debug channel). Error: %d.", i, errno);

			libusb_free_transfer(handle->fx3Support.debugTransfers[i]);
			handle->fx3Support.debugTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		handle->fx3Support.debugTransfers[i]->dev_handle = handle->usbState.deviceHandle;
		handle->fx3Support.debugTransfers[i]->endpoint   = DEBUG_ENDPOINT;
		handle->fx3Support.debugTransfers[i]->type       = LIBUSB_TRANSFER_TYPE_INTERRUPT;
		handle->fx3Support.debugTransfers[i]->callback   = &libUsbDebugCallback;
		handle->fx3Support.debugTransfers[i]->user_data  = handle;
		handle->fx3Support.debugTransfers[i]->timeout    = 0;
		handle->fx3Support.debugTransfers[i]->flags      = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(handle->fx3Support.debugTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_fetch_add(&handle->fx3Support.activeDebugTransfers, 1);
		}
		else {
			davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
				"Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(handle->fx3Support.debugTransfers[i]);
			handle->fx3Support.debugTransfers[i] = NULL;
		}
	}

	if (atomic_load(&handle->fx3Support.activeDebugTransfers) == 0) {
		// Didn't manage to allocate any USB transfers, log failure.
		davisLog(CAER_LOG_CRITICAL, &handle->cHandle, "Unable to allocate any libusb transfers (debug channel).");
	}
}

static void cancelAndDeallocateDebugTransfers(davisHandle handle) {
	// Wait for all transfers to go away.
	struct timespec waitForTerminationSleep = {.tv_sec = 0, .tv_nsec = 1000000};

	while (atomic_load(&handle->fx3Support.activeDebugTransfers) > 0) {
		// Continue trying to cancel all transfers until there are none left.
		// It seems like one cancel pass is not enough and some hang around.
		for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
			if (handle->fx3Support.debugTransfers[i] != NULL) {
				errno = libusb_cancel_transfer(handle->fx3Support.debugTransfers[i]);
				if ((errno != LIBUSB_SUCCESS) && (errno != LIBUSB_ERROR_NOT_FOUND)) {
					davisLog(CAER_LOG_CRITICAL, &handle->cHandle,
						"Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i,
						libusb_strerror(errno), errno);
					// Proceed with trying to cancel all transfers regardless of errors.
				}
			}
		}

		// Sleep for 1ms to avoid busy loop.
		thrd_sleep(&waitForTerminationSleep, NULL);
	}

	// No more transfers in flight, deallocate them all here.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		if (handle->fx3Support.debugTransfers[i] != NULL) {
			libusb_free_transfer(handle->fx3Support.debugTransfers[i]);
			handle->fx3Support.debugTransfers[i] = NULL;
		}
	}
}

static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer) {
	davisHandle handle = transfer->user_data;

	// Completed or cancelled transfers are what we expect to handle here, so
	// if they do have data attached, try to parse them.
	if (((transfer->status == LIBUSB_TRANSFER_COMPLETED) || (transfer->status == LIBUSB_TRANSFER_CANCELLED))
		&& (transfer->actual_length > 0)) {
		// Handle debug data.
		debugTranslator(handle, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter and exiting.
	// Freeing the transfers is taken care of by cancelAndDeallocateDebugTransfers().
	atomic_fetch_sub(&handle->fx3Support.activeDebugTransfers, 1);
}

static void debugTranslator(davisHandle handle, const uint8_t *buffer, size_t bytesSent) {
	// Check if this is a debug message (length 7-64 bytes).
	if ((bytesSent >= 7) && (buffer[0] == 0x00)) {
		// Debug message, log this.
		davisLog(CAER_LOG_ERROR, &handle->cHandle, "Error message: '%s' (code %u at time %u).", &buffer[6], buffer[1],
			*((const uint32_t *) &buffer[2]));
	}
	else {
		// Unknown/invalid debug message, log this.
		davisLog(CAER_LOG_WARNING, &handle->cHandle, "Unknown/invalid debug message.");
	}
}

// External functions defined in main davis.h.
struct caer_davis_info caerDavisInfoGet(caerDeviceHandle cdh) {
	davisCommonHandle handle = (davisCommonHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_davis_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Check if device type is supported.
	if ((handle->deviceType != CAER_DEVICE_DAVIS) && (handle->deviceType != CAER_DEVICE_DAVIS_FX2)
		&& (handle->deviceType != CAER_DEVICE_DAVIS_FX3) && (handle->deviceType != CAER_DEVICE_DAVIS_RPI)) {
		struct caer_davis_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool caerDavisROIConfigure(caerDeviceHandle cdh, uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY) {
	davisCommonHandle handle = (davisCommonHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if ((handle->deviceType != CAER_DEVICE_DAVIS) && (handle->deviceType != CAER_DEVICE_DAVIS_FX2)
		&& (handle->deviceType != CAER_DEVICE_DAVIS_FX3) && (handle->deviceType != CAER_DEVICE_DAVIS_RPI)) {
		return (false);
	}

	// Check that end >= start.
	if ((startX > endX) || (startY > endY)) {
		return (false);
	}

	// 5 commands always (disable, four coordinates).
	uint16_t commandsNumber = 5;

	// 6 commands if enable also has to be sent.
	uint32_t isEnabled = 0;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, &isEnabled);

	if (isEnabled) {
		commandsNumber++;
	}

	// First disable, then set all four coordinates, then enable again IF requested.
	struct spi_config_params spiMultiConfig[commandsNumber];

	for (size_t i = 0; i < commandsNumber; i++) {
		spiMultiConfig[i].moduleAddr = DAVIS_CONFIG_APS;

		switch (i) {
			case 0: // Disable.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_RUN;
				spiMultiConfig[i].param     = false;
				break;

			case 1: // StartX.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_START_COLUMN_0;
				spiMultiConfig[i].param     = startX;
				break;

			case 2: // StartY.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_START_ROW_0;
				spiMultiConfig[i].param     = startY;
				break;

			case 3: // EndX.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_END_COLUMN_0;
				spiMultiConfig[i].param     = endX;
				break;

			case 4: // EndY.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_END_ROW_0;
				spiMultiConfig[i].param     = endY;
				break;

			case 5: // Enable.
				spiMultiConfig[i].paramAddr = DAVIS_CONFIG_APS_RUN;
				spiMultiConfig[i].param     = true;
				break;
		}
	}

	return (spiConfigSendMultiple(handle->spiConfigPtr, spiMultiConfig, commandsNumber));
}

uint16_t caerBiasVDACGenerate(const struct caer_bias_vdac vdacBias) {
	// Build up bias value from all its components.
	uint16_t biasValue = U16T((vdacBias.voltageValue & 0x3F) << 0);
	biasValue          = U16T(biasValue | ((vdacBias.currentValue & 0x07) << 6));

	return (biasValue);
}

struct caer_bias_vdac caerBiasVDACParse(const uint16_t vdacBias) {
	struct caer_bias_vdac biasValue;

	// Decompose bias integer into its parts.
	biasValue.voltageValue = vdacBias & 0x3F;
	biasValue.currentValue = U16T(vdacBias >> 6) & 0x07;

	return (biasValue);
}

uint16_t caerBiasCoarseFineGenerate(const struct caer_bias_coarsefine coarseFineBias) {
	uint16_t biasValue = 0;

	// Build up bias value from all its components.
	if (coarseFineBias.enabled) {
		biasValue |= 0x01U;
	}
	if (coarseFineBias.sexN) {
		biasValue |= 0x02U;
	}
	if (coarseFineBias.typeNormal) {
		biasValue |= 0x04U;
	}
	if (coarseFineBias.currentLevelNormal) {
		biasValue |= 0x08U;
	}

	biasValue = U16T(biasValue | ((coarseFineBias.fineValue & 0xFF) << 4));
	biasValue = U16T(biasValue | ((coarseFineBias.coarseValue & 0x07) << 12));

	return (biasValue);
}

struct caer_bias_coarsefine caerBiasCoarseFineParse(const uint16_t coarseFineBias) {
	struct caer_bias_coarsefine biasValue;

	// Decompose bias integer into its parts.
	biasValue.enabled            = (coarseFineBias & 0x01);
	biasValue.sexN               = (coarseFineBias & 0x02);
	biasValue.typeNormal         = (coarseFineBias & 0x04);
	biasValue.currentLevelNormal = (coarseFineBias & 0x08);
	biasValue.fineValue          = U8T(coarseFineBias >> 4) & 0xFF;
	biasValue.coarseValue        = U8T(coarseFineBias >> 12) & 0x07;

	return (biasValue);
}

static const uint32_t coarseCurrents[8] = {
	11,
	94,
	756,
	6054,
	48437,
	387500,
	3100000,
	24800000,
};

struct caer_bias_coarsefine caerBiasCoarseFineFromCurrent(uint32_t picoAmps) {
	struct caer_bias_coarsefine biasValue;

	// Disable bias.
	if (picoAmps == 0) {
		biasValue.coarseValue = 0;
		biasValue.fineValue   = 0;

		return (biasValue);
	}

	// We support between 1 pA and 24.8 uA (24.8 mio pA).
	if (picoAmps > 24800000) {
		picoAmps = 24800000; // Limit to 24.8 uA.
	}

	// Select appropriate coarse value from precomputed table.
	uint8_t coarseValue = 0;

	for (uint8_t i = 0; i < 8; i++) {
		if (picoAmps <= coarseCurrents[i]) {
			coarseValue = i;
			break;
		}
	}

	biasValue.coarseValue = coarseValue;

	// Calculate coarse current value based on value going to device.
	// This is the maximum for the fine divider.
	double coarseCurrent = (double) coarseCurrents[coarseValue];

	double fineValue = round(((255.0 * (double) picoAmps) / coarseCurrent));

	// Ensure within range.
	if (fineValue < 1) {
		fineValue = 1;
	}
	else if (fineValue > 255) {
		fineValue = 255;
	}

	biasValue.fineValue = U8T(fineValue);

	return (biasValue);
}

uint32_t caerBiasCoarseFineToCurrent(struct caer_bias_coarsefine coarseFineBias) {
	// Special base: disabled bias.
	if (coarseFineBias.fineValue == 0) {
		return (0);
	}

	double coarseCurrent = (double) coarseCurrents[coarseFineBias.coarseValue];
	double fineCurrent   = (coarseCurrent * (double) coarseFineBias.fineValue) / 255.0;

	double biasCurrent = round(fineCurrent);

	return (U32T(biasCurrent));
}

uint16_t caerBiasShiftedSourceGenerate(const struct caer_bias_shiftedsource shiftedSourceBias) {
	uint16_t biasValue = 0;

	if (shiftedSourceBias.operatingMode == HI_Z) {
		biasValue |= 0x01U;
	}
	else if (shiftedSourceBias.operatingMode == TIED_TO_RAIL) {
		biasValue |= 0x02U;
	}

	if (shiftedSourceBias.voltageLevel == SINGLE_DIODE) {
		biasValue |= (0x01U << 2);
	}
	else if (shiftedSourceBias.voltageLevel == DOUBLE_DIODE) {
		biasValue |= (0x02U << 2);
	}

	biasValue = U16T(biasValue | ((shiftedSourceBias.refValue & 0x3F) << 4));
	biasValue = U16T(biasValue | ((shiftedSourceBias.regValue & 0x3F) << 10));

	return (biasValue);
}

struct caer_bias_shiftedsource caerBiasShiftedSourceParse(const uint16_t shiftedSourceBias) {
	struct caer_bias_shiftedsource biasValue;

	// Decompose bias integer into its parts.
	if (shiftedSourceBias & 0x01) {
		biasValue.operatingMode = HI_Z;
	}
	else if (shiftedSourceBias & 0x02) {
		biasValue.operatingMode = TIED_TO_RAIL;
	}
	else {
		biasValue.operatingMode = SHIFTED_SOURCE;
	}

	if (U16T(shiftedSourceBias >> 2) & 0x01) {
		biasValue.voltageLevel = SINGLE_DIODE;
	}
	else if (U16T(shiftedSourceBias >> 2) & 0x02) {
		biasValue.voltageLevel = DOUBLE_DIODE;
	}
	else {
		biasValue.voltageLevel = SPLIT_GATE;
	}

	biasValue.refValue = U16T(shiftedSourceBias >> 4) & 0x3F;
	biasValue.regValue = U16T(shiftedSourceBias >> 10) & 0x3F;

	return (biasValue);
}
