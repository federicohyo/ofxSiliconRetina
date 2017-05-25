#include "davis_common.h"

static void davisEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent);
static void davisTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param);

void davisCommonLog(enum caer_log_level logLevel, davisHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static inline void checkStrictMonotonicTimestamp(davisHandle handle) {
	if (handle->state.currentTimestamp <= handle->state.lastTimestamp) {
		davisCommonLog(CAER_LOG_ALERT, handle,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void updateROISizes(davisState state) {
	// Calculate APS ROI sizes for each region.
	for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
		uint16_t startColumn = state->apsROIPositionX[i];
		uint16_t startRow = state->apsROIPositionY[i];
		uint16_t endColumn = state->apsROISizeX[i];
		uint16_t endRow = state->apsROISizeY[i];

		// Position is already set to startCol/Row, so we don't have to reset
		// it here. We only have to calculate size from start and end Col/Row.
		if (startColumn < state->apsSizeX && endColumn < state->apsSizeX && startRow < state->apsSizeY
			&& endRow < state->apsSizeY) {
			state->apsROISizeX[i] = U16T(endColumn + 1 - startColumn);
			state->apsROISizeY[i] = U16T(endRow + 1 - startRow);

			if (state->apsInvertXY) {
				// Inverted, so X[StartColumn] becomes endColumn. Y[endRow] becomes startRow.
				// Same accounting for origin in upper left corner, but on the other axis here.
				state->apsROIPositionX[i] = U16T(state->apsSizeX - 1 - endColumn);
				state->apsROIPositionY[i] = startRow;
			}
			else {
				// Y position needs to be inverted with endRow to account for the
				// origin (0, 0) being in the upper left corner. X is fine as startColumn.
				state->apsROIPositionY[i] = U16T(state->apsSizeY - 1 - endRow);
			}
		}
		else {
			// Turn off this ROI region.
			state->apsROISizeX[i] = state->apsROIPositionX[i] = U16T(state->apsSizeX);
			state->apsROISizeY[i] = state->apsROIPositionY[i] = U16T(state->apsSizeY);
		}
	}
}

static inline void initFrame(davisHandle handle) {
	davisState state = &handle->state;

	state->apsCurrentReadoutType = APS_READOUT_RESET;
	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		state->apsCountX[i] = 0;
		state->apsCountY[i] = 0;
	}

	// for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
	memset(state->currentFrameEvent[0], 0, (sizeof(struct caer_frame_event) - sizeof(uint16_t)));
	// }

	if (state->apsROIUpdate != 0) {
		updateROISizes(state);
	}

	// Skip frame if ROI region is disabled.
	if (state->apsROIPositionX[0] >= state->apsSizeX || state->apsROIPositionY[0] >= state->apsSizeY) {
		return;
	}

	// Write out start of frame timestamp.
	caerFrameEventSetTSStartOfFrame(state->currentFrameEvent[0], state->currentTimestamp);

	// Send APS info event out (as special event).
	caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
		state->currentSpecialPacketPosition);
	caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
	caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_START);
	caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
	state->currentSpecialPacketPosition++;

	// Setup frame. Only ROI region 0 is supported currently.
	caerFrameEventSetColorFilter(state->currentFrameEvent[0], handle->info.apsColorFilter);
	caerFrameEventSetROIIdentifier(state->currentFrameEvent[0], 0);
	caerFrameEventSetLengthXLengthYChannelNumber(state->currentFrameEvent[0], state->apsROISizeX[0],
		state->apsROISizeY[0], APS_ADC_CHANNELS, state->currentFramePacket);
	caerFrameEventSetPositionX(state->currentFrameEvent[0], state->apsROIPositionX[0]);
	caerFrameEventSetPositionY(state->currentFrameEvent[0], state->apsROIPositionY[0]);
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +-2 g - 16384 LSB/g
	// 1 - +-4 g - 8192 LSB/g
	// 2 - +-8 g - 4096 LSB/g
	// 3 - +-16 g - 2048 LSB/g
	float accelScale = 65536.0f / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Gyroscope scale is:
	// 0 - +-250 °/s - 131 LSB/°/s
	// 1 - +-500 °/s - 65.5 LSB/°/s
	// 2 - +-1000 °/s - 32.8 LSB/°/s
	// 3 - +-2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0f / (float) U32T(500 * (1 << imuGyroScale));

	return (gyroScale);
}

static inline void freeAllDataMemory(davisState state) {
	if (state->dataExchangeBuffer != NULL) {
		ringBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPolarityPacket != NULL) {
		free(&state->currentPolarityPacket->packetHeader);
		state->currentPolarityPacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT, NULL);
		}
	}

	if (state->currentSpecialPacket != NULL) {
		free(&state->currentSpecialPacket->packetHeader);
		state->currentSpecialPacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT, NULL);
		}
	}

	if (state->currentFramePacket != NULL) {
		free(&state->currentFramePacket->packetHeader);
		state->currentFramePacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, FRAME_EVENT, NULL);
		}
	}

	if (state->currentIMU6Packet != NULL) {
		free(&state->currentIMU6Packet->packetHeader);
		state->currentIMU6Packet = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, IMU6_EVENT, NULL);
		}
	}

	if (state->currentSamplePacket != NULL) {
		free(&state->currentSamplePacket->packetHeader);
		state->currentSamplePacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DAVIS_SAMPLE_POSITION, NULL);
		}
	}

	if (state->currentPacketContainer != NULL) {
		caerEventPacketContainerFree(state->currentPacketContainer);
		state->currentPacketContainer = NULL;
	}

	if (state->apsCurrentResetFrame != NULL) {
		free(state->apsCurrentResetFrame);
		state->apsCurrentResetFrame = NULL;
	}

	// Also free current ROI frame events.
	free(state->currentFrameEvent[0]); // Other regions are contained within contiguous memory block.

	for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
		// Reset pointers to NULL.
		state->currentFrameEvent[i] = NULL;
	}
}

bool davisCommonOpen(davisHandle handle, uint16_t VID, uint16_t PID, const char *deviceName, uint16_t deviceID,
	uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict,
	uint16_t requiredLogicRevision, uint16_t requiredFirmwareVersion) {
	davisState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	atomic_store(&state->dataExchangeBufferSize, 64);
	atomic_store(&state->dataExchangeBlocking, false);
	atomic_store(&state->dataExchangeStartProducers, true);
	atomic_store(&state->dataExchangeStopProducers, true);

	// Packet settings (size (in events) and time interval (in µs)).
	atomic_store(&state->maxPacketContainerPacketSize, 8192);
	atomic_store(&state->maxPacketContainerInterval, 10000);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	atomic_store(&state->usbState.usbLogLevel, globalLogLevel);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, deviceName, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);

	// Try to open a DAVIS device on a specific USB port.
	if (!usbDeviceOpen(&state->usbState, VID, PID, busNumberRestrict, devAddressRestrict, serialNumberRestrict,
		requiredLogicRevision, requiredFirmwareVersion)) {
		return (false);
	}

	struct usb_info usbInfo = usbGenerateInfo(&state->usbState, deviceName, deviceID);
	if (usbInfo.deviceString == NULL) {
		usbDeviceClose(&state->usbState);

		return (false);
	}

	// Setup USB.
	usbSetDataCallback(&state->usbState, &davisEventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);

		free(usbInfo.deviceString);

		return (false);
	}

	// Populate info variables based on data from device.
	uint32_t param32 = 0;

	handle->info.deviceID = I16T(deviceID);
	strncpy(handle->info.deviceSerialNumber, usbInfo.serialNumber, 8 + 1);
	handle->info.deviceUSBBusNumber = usbInfo.busNumber;
	handle->info.deviceUSBDeviceAddress = usbInfo.devAddress;
	handle->info.deviceString = usbInfo.deviceString;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_VERSION, &param32);
	handle->info.logicVersion = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	handle->info.logicClock = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_ADC_CLOCK, &param32);
	handle->info.adcClock = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
	handle->info.chipID = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_PIXEL_FILTER, &param32);
	handle->info.dvsHasPixelFilter = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_BACKGROUND_ACTIVITY_FILTER, &param32);
	handle->info.dvsHasBackgroundActivityFilter = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_TEST_EVENT_GENERATOR, &param32);
	handle->info.dvsHasTestEventGenerator = param32;

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLOR_FILTER, &param32);
	handle->info.apsColorFilter = U8T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_GLOBAL_SHUTTER, &param32);
	handle->info.apsHasGlobalShutter = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_QUAD_ROI, &param32);
	handle->info.apsHasQuadROI = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_EXTERNAL_ADC, &param32);
	handle->info.apsHasExternalADC = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_INTERNAL_ADC, &param32);
	handle->info.apsHasInternalADC = param32;

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_HAS_GENERATOR, &param32);
	handle->info.extInputHasGenerator = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_HAS_EXTRA_DETECTORS, &param32);
	handle->info.extInputHasExtraDetectors = param32;

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_COLUMNS, &param32);
	state->dvsSizeX = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_ROWS, &param32);
	state->dvsSizeY = I16T(param32);

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ORIENTATION_INFO, &param32);
	state->dvsInvertXY = param32 & 0x04;

	if (state->dvsInvertXY) {
		handle->info.dvsSizeX = state->dvsSizeY;
		handle->info.dvsSizeY = state->dvsSizeX;
	}
	else {
		handle->info.dvsSizeX = state->dvsSizeX;
		handle->info.dvsSizeY = state->dvsSizeY;
	}

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_COLUMNS, &param32);
	state->apsSizeX = I16T(param32);
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_ROWS, &param32);
	state->apsSizeY = I16T(param32);

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ORIENTATION_INFO, &param32);
	state->apsInvertXY = param32 & 0x04;
	state->apsFlipX = param32 & 0x02;
	state->apsFlipY = param32 & 0x01;

	if (state->apsInvertXY) {
		handle->info.apsSizeX = state->apsSizeY;
		handle->info.apsSizeY = state->apsSizeX;
	}
	else {
		handle->info.apsSizeX = state->apsSizeX;
		handle->info.apsSizeY = state->apsSizeY;
	}

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ORIENTATION_INFO, &param32);
	state->imuFlipX = param32 & 0x04;
	state->imuFlipY = param32 & 0x02;
	state->imuFlipZ = param32 & 0x01;

	davisCommonLog(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		usbInfo.busNumber, usbInfo.devAddress);

	return (true);
}

bool davisCommonClose(davisHandle handle) {
	davisState state = &handle->state;

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	davisCommonLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	return (true);
}

struct caer_davis_info caerDavisInfoGet(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_davis_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DAVIS_FX2 && handle->deviceType != CAER_DEVICE_DAVIS_FX3) {
		struct caer_davis_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool davisCommonSendDefaultFPGAConfig(caerDeviceHandle cdh,
bool (*configSet)(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param)) {
	davisHandle handle = (davisHandle) cdh;

	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, false);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL, true);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL, false);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL, false);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
	(*configSet)(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL, false);

	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_ROW, 4); // in cycles @ LogicClock
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN, 0); // in cycles @ LogicClock
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW, 1); // in cycles @ LogicClock
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN, 0); // in cycles @ LogicClock
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL, false);
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS, true);
	(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL, false);
	if (handle->info.dvsHasPixelFilter) {
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, U32T(handle->info.dvsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, U32T(handle->info.dvsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, U32T(handle->info.dvsSizeX));
	}
	if (handle->info.dvsHasBackgroundActivityFilter) {
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, true);
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_DELTAT, 20000);	// in µs
	}
	if (handle->info.dvsHasTestEventGenerator) {
		(*configSet)(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE, false);
	}

	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ, true);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL, true);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, handle->info.apsHasGlobalShutter);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, 0);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, 0);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, U16T(handle->info.apsSizeX - 1));
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, U16T(handle->info.apsSizeY - 1));
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4000); // in µs, converted to cycles @ ADCClock later
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_DELAY, 1000); // in µs, converted to cycles @ ADCClock later
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_SETTLE, U32T(handle->info.adcClock / 3)); // in cycles @ ADCClock
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLUMN_SETTLE, U32T(handle->info.adcClock)); // in cycles @ ADCClock
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROW_SETTLE, U32T(handle->info.adcClock / 3)); // in cycles @ ADCClock
	(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_NULL_SETTLE, U32T(handle->info.adcClock / 10)); // in cycles @ ADCClock
	if (handle->info.apsHasQuadROI) {
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_1, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_1, U32T(handle->info.apsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_1, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_1, U32T(handle->info.apsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_2, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_2, U32T(handle->info.apsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_2, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_2, U32T(handle->info.apsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_3, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_3, U32T(handle->info.apsSizeY));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_3, U32T(handle->info.apsSizeX));
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_3, U32T(handle->info.apsSizeY));
	}
	if (handle->info.apsHasInternalADC) {
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_USE_INTERNAL_ADC, true);
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_ENABLE, true);
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_SETTLE, U32T(handle->info.adcClock)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_RESET, U32T(handle->info.adcClock / 3)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_SHORT_RESET, false);
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ADC_TEST_MODE, false);
	}
	if (IS_DAVISRGB(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_TRANSFER, U32T(handle->info.adcClock * 25)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_RSFDSETTLE, U32T(handle->info.adcClock * 15)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSPDRESET, U32T(handle->info.adcClock * 15)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSRESETFALL, U32T(handle->info.adcClock * 15)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSTXFALL, U32T(handle->info.adcClock * 15)); // in cycles @ ADCClock
		(*configSet)(cdh, DAVIS_CONFIG_APS, DAVISRGB_CONFIG_APS_GSFDRESET, U32T(handle->info.adcClock * 15)); // in cycles @ ADCClock
	}

	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_TEMP_STANDBY, false);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_STANDBY, false);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_STANDBY, false);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_CYCLE, false);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_WAKEUP, 1);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, 1);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, 1);
	(*configSet)(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, 1);

	(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, false);
	(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, false);
	(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, true);
	(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, true);
	(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH, U32T(handle->info.logicClock)); // in cycles @ LogicClock

	(*configSet)(cdh, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_RUN, false); // Microphones disabled by default.
	(*configSet)(cdh, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY, 32); // 48 KHz sampling frequency.

	if (handle->info.extInputHasGenerator) {
		// Disable generator by default. Has to be enabled manually after sendDefaultConfig() by user!
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, true);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
			U32T(handle->info.logicClock / 2)); // in cycles @ LogicClock
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);
	}

	if (handle->info.extInputHasExtraDetectors) {
		// Disable extra detectors by default. Have to be enabled manually after sendDefaultConfig() by user!
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1, true);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1, true);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock

		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2, false);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2, true);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2, true);
		(*configSet)(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock
	}

	(*configSet)(cdh, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs time-slices (defaults to 1ms)

	return (true);
}

#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_N_TYPE_CAS(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, \
	.typeNormal = false, .currentLevelNormal = true }

/*
 * #define CF_P_TYPE_CAS(COARSE, FINE) (struct caer_bias_coarsefine) \
 *	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, \
 *	.typeNormal = false, .currentLevelNormal = true }
 */

#define CF_N_TYPE_OFF(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = true, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE_OFF(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = false, \
	.typeNormal = true, .currentLevelNormal = true }

#define SHIFTSOURCE(REF, REG, OPMODE) (struct caer_bias_shiftedsource) \
	{ .refValue = REF, .regValue = REG, .operatingMode = OPMODE, .voltageLevel = SPLIT_GATE }

#define VDAC(VOLT, CURR) (struct caer_bias_vdac) \
	{ .voltageValue = VOLT, .currentValue = CURR }

bool davisCommonSendDefaultChipConfig(caerDeviceHandle cdh,
bool (*configSet)(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param)) {
	davisHandle handle = (davisHandle) cdh;

	// Default bias configuration.
	if (IS_DAVIS240(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 39)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 0)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSCASEPC,
			caerBiasCoarseFineGenerate(CF_N_TYPE_CAS(5, 185)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFCASBNC,
			caerBiasCoarseFineGenerate(CF_N_TYPE_CAS(5, 115)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSROSFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LOCALBUFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PIXINVBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 58)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 16)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 25)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUXBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUYBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFTHRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFREFRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PADFOLLBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSOVERFLOWLEVELBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 253)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_BIASBUFFER,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));

	}

	if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID)
	|| IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL, caerBiasVDACGenerate(VDAC(27, 6)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSCAS, caerBiasVDACGenerate(VDAC(21, 6)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFHIGH, caerBiasVDACGenerate(VDAC(30, 7)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFLOW, caerBiasVDACGenerate(VDAC(1, 7)));

		if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
			// Only DAVIS346 and 640 have ADC testing.
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
				caerBiasVDACGenerate(VDAC(21, 7)));
		}

		if (IS_DAVIS208(handle->info.chipID)) {
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_RESETHIGHPASS, caerBiasVDACGenerate(VDAC(63, 7)));
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSS, caerBiasVDACGenerate(VDAC(11, 5)));

			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REGBIASBP,
				caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSSBN,
				caerBiasCoarseFineGenerate(CF_N_TYPE(5, 20)));
		}

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LOCALBUFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PADFOLLBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 39)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 1)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PIXINVBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 58)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 16)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 25)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_READOUTBUFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSROSFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCCOMPBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_COLSELLOWBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DACBUFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUXBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUYBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFREFRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFTHRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_BIASBUFFER,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));

		if (IS_DAVIS640(handle->info.chipID)) {
			// Slow down pixels for big 640x480 array, to avoid overwhelming the AER bus.
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_PRBP,
				caerBiasCoarseFineGenerate(CF_P_TYPE(2, 3)));
			(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_PRSFBP,
				caerBiasCoarseFineGenerate(CF_P_TYPE(1, 1)));
		}
	}

	if (IS_DAVISRGB(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_APSCAS, caerBiasVDACGenerate(VDAC(21, 4)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_OVG1LO, caerBiasVDACGenerate(VDAC(21, 4)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_OVG2LO, caerBiasVDACGenerate(VDAC(0, 0)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_TX2OVG2HI, caerBiasVDACGenerate(VDAC(63, 0)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_GND07, caerBiasVDACGenerate(VDAC(13, 4)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ADCTESTVOLTAGE, caerBiasVDACGenerate(VDAC(21, 0)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ADCREFHIGH, caerBiasVDACGenerate(VDAC(63, 7)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ADCREFLOW, caerBiasVDACGenerate(VDAC(0, 7)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_IFREFRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_IFTHRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_LOCALBUFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 164)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_PADFOLLBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(7, 209)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_PIXINVBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(4, 164)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 54)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 63)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(2, 138)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 108)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_PRSFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(1, 108)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 28)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ARRAYBIASBUFFERBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 128)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ARRAYLOGICBUFFERBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_FALLTIMEBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(7, 41)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_RISETIMEBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(6, 162)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_READOUTBUFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE_OFF(6, 20)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_APSROSFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 255)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_ADCCOMPBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(4, 159)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_DACBUFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(6, 194)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_AEPUXBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_AEPUYBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_BIASBUFFER,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 251)));

		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, TIED_TO_RAIL)));
		(*configSet)(cdh, DAVIS_CONFIG_BIAS, DAVISRGB_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(2, 33, SHIFTED_SOURCE)));
	}

	// Default chip configuration.
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX0, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX1, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX2, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX3, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX0, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX1, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX2, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_BIASMUX0, 0);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETCALIBNEURON, true);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON, false);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETTESTPIXEL, true);
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_AERNAROW, false);  // Use nArow in the AER state machine.
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_USEAOUT, false); // Enable analog pads for aMUX output (testing).

	// No GlobalShutter flag set here, we already set it above for the APS GS flag,
	// and that is automatically propagated to the chip config shift-register in
	// configSet() and kept in sync.

	// Special extra pixels control for DAVIS240 A/B.
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL, false);

	// Select which gray counter to use with the internal ADC: '0' means the external gray counter is used, which
	// has to be supplied off-chip. '1' means the on-chip gray counter is used instead.
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER, 1);

	// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
	// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
	(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS346_CONFIG_CHIP_TESTADC, false);

	if (IS_DAVIS208(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG, false);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTBIASREFSS, false);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTSENSE, true);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPOSFB, false);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTHIGHPASS, false);
	}

	if (IS_DAVISRGB(handle->info.chipID)) {
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVISRGB_CONFIG_CHIP_ADJUSTOVG1LO, true);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVISRGB_CONFIG_CHIP_ADJUSTOVG2LO, false);
		(*configSet)(cdh, DAVIS_CONFIG_CHIP, DAVISRGB_CONFIG_CHIP_ADJUSTTX2OVG2HI, false);
	}

	return (true);
}

bool davisCommonConfigSet(davisHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	davisState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
					usbSetTransfersNumber(&state->usbState, param);
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					usbSetTransfersSize(&state->usbState, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
					atomic_store(&state->dataExchangeBufferSize, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
					atomic_store(&state->dataExchangeBlocking, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
					atomic_store(&state->dataExchangeStartProducers, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
					atomic_store(&state->dataExchangeStopProducers, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_PACKETS:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE:
					atomic_store(&state->maxPacketContainerPacketSize, param);
					break;

				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL:
					atomic_store(&state->maxPacketContainerInterval, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					atomic_store(&state->deviceLogLevel, U8T(param));

					// Set USB log-level to this value too.
					atomic_store(&state->usbState.usbLogLevel, U8T(param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_MUX:
			switch (paramAddr) {
				case DAVIS_CONFIG_MUX_RUN:
				case DAVIS_CONFIG_MUX_TIMESTAMP_RUN:
				case DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						uint8_t spiMultiConfig[6 + 6] = { 0 };

						spiMultiConfig[0] = DAVIS_CONFIG_MUX;
						spiMultiConfig[1] = DAVIS_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[2] = 0x00;
						spiMultiConfig[3] = 0x00;
						spiMultiConfig[4] = 0x00;
						spiMultiConfig[5] = 0x01;

						spiMultiConfig[6] = DAVIS_CONFIG_MUX;
						spiMultiConfig[7] = DAVIS_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[8] = 0x00;
						spiMultiConfig[9] = 0x00;
						spiMultiConfig[10] = 0x00;
						spiMultiConfig[11] = 0x00;

						return (usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, 2, 0,
							spiMultiConfig, sizeof(spiMultiConfig)));
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_DVS:
			switch (paramAddr) {
				case DAVIS_CONFIG_DVS_RUN:
				case DAVIS_CONFIG_DVS_ACK_DELAY_ROW:
				case DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN:
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
					if (handle->info.dvsHasPixelFilter) {
						if (handle->state.dvsInvertXY) {
							// Convert to column if X/Y inverted.
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, U8T(paramAddr + 1), param));
						}
						else {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
						}
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						if (handle->state.dvsInvertXY) {
							// Convert to row if X/Y inverted.
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, U8T(paramAddr - 1), param));
						}
						else {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
						}
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_DELTAT:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE:
					if (handle->info.dvsHasTestEventGenerator) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_APS:
			switch (paramAddr) {
				case DAVIS_CONFIG_APS_RUN:
				case DAVIS_CONFIG_APS_RESET_READ:
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_ROW_SETTLE:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_RESET_SETTLE:
				case DAVIS_CONFIG_APS_COLUMN_SETTLE:
				case DAVIS_CONFIG_APS_NULL_SETTLE:
					// Not supported on DAVIS RGB APS state machine.
					if (!IS_DAVISRGB(handle->info.chipID)) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_0:
					if (state->apsInvertXY) {
						// Convert to row if X/Y inverted.
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_START_ROW_0, param));
					}
					else {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_START_COLUMN_0, param));
					}
					break;

				case DAVIS_CONFIG_APS_START_ROW_0:
					if (state->apsInvertXY) {
						// Convert to column if X/Y inverted.
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_END_COLUMN_0,
						U32T(state->apsSizeX) - 1 - param));
					}
					else {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_END_ROW_0,
						U32T(state->apsSizeY) - 1 - param));
					}
					break;

				case DAVIS_CONFIG_APS_END_COLUMN_0:
					if (state->apsInvertXY) {
						// Convert to row if X/Y inverted.
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_END_ROW_0, param));
					}
					else {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_END_COLUMN_0, param));
					}
					break;

				case DAVIS_CONFIG_APS_END_ROW_0:
					if (state->apsInvertXY) {
						// Convert to column if X/Y inverted.
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_START_COLUMN_0,
						U32T(state->apsSizeX) - 1 - param));
					}
					else {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
						DAVIS_CONFIG_APS_START_ROW_0,
						U32T(state->apsSizeY) - 1 - param));
					}
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Exposure and Frame Delay are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					if (!atomic_load(&state->apsAutoExposureEnabled)) {
						state->apsExposureLastSetValue = param;
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr,
							param * U16T(handle->info.adcClock)));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_FRAME_DELAY:
					// Exposure and Frame Delay are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr,
						param * U16T(handle->info.adcClock)));
					break;

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						// Keep in sync with chip config module GlobalShutter parameter.
						if (!spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP,
						DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER, param)) {
							return (false);
						}

						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_1:
				case DAVIS_CONFIG_APS_END_COLUMN_1:
				case DAVIS_CONFIG_APS_START_COLUMN_2:
				case DAVIS_CONFIG_APS_END_COLUMN_2:
				case DAVIS_CONFIG_APS_START_COLUMN_3:
				case DAVIS_CONFIG_APS_END_COLUMN_3:
				case DAVIS_CONFIG_APS_START_ROW_1:
				case DAVIS_CONFIG_APS_END_ROW_1:
				case DAVIS_CONFIG_APS_START_ROW_2:
				case DAVIS_CONFIG_APS_END_ROW_2:
				case DAVIS_CONFIG_APS_START_ROW_3:
				case DAVIS_CONFIG_APS_END_ROW_3:
					// TODO: no support on host-side for QuadROI and multi-frame decoding.
					return (false);
					break;

				case DAVIS_CONFIG_APS_USE_INTERNAL_ADC:
				case DAVIS_CONFIG_APS_SAMPLE_ENABLE:
				case DAVIS_CONFIG_APS_SAMPLE_SETTLE:
				case DAVIS_CONFIG_APS_RAMP_RESET:
				case DAVIS_CONFIG_APS_RAMP_SHORT_RESET:
				case DAVIS_CONFIG_APS_ADC_TEST_MODE:
					if (handle->info.apsHasInternalADC) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVISRGB_CONFIG_APS_TRANSFER:
				case DAVISRGB_CONFIG_APS_RSFDSETTLE:
				case DAVISRGB_CONFIG_APS_GSPDRESET:
				case DAVISRGB_CONFIG_APS_GSRESETFALL:
				case DAVISRGB_CONFIG_APS_GSTXFALL:
				case DAVISRGB_CONFIG_APS_GSFDRESET:
					// Support for DAVISRGB extra timing parameters.
					if (IS_DAVISRGB(handle->info.chipID)) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SNAPSHOT: {
					// Use multi-command VR for more efficient implementation of snapshot,
					// that also guarantees returning to the default state (not running).
					if (param) {
						uint8_t spiMultiConfig[6 + 6] = { 0 };

						spiMultiConfig[0] = DAVIS_CONFIG_APS;
						spiMultiConfig[1] = DAVIS_CONFIG_APS_RUN;
						spiMultiConfig[2] = 0x00;
						spiMultiConfig[3] = 0x00;
						spiMultiConfig[4] = 0x00;
						spiMultiConfig[5] = 0x01;

						spiMultiConfig[6] = DAVIS_CONFIG_APS;
						spiMultiConfig[7] = DAVIS_CONFIG_APS_RUN;
						spiMultiConfig[8] = 0x00;
						spiMultiConfig[9] = 0x00;
						spiMultiConfig[10] = 0x00;
						spiMultiConfig[11] = 0x00;

						return (usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, 2, 0,
							spiMultiConfig, sizeof(spiMultiConfig)));
					}
					break;
				}

				case DAVIS_CONFIG_APS_AUTOEXPOSURE:
					atomic_store(&state->apsAutoExposureEnabled, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN:
				case DAVIS_CONFIG_IMU_TEMP_STANDBY:
				case DAVIS_CONFIG_IMU_ACCEL_STANDBY:
				case DAVIS_CONFIG_IMU_GYRO_STANDBY:
				case DAVIS_CONFIG_IMU_LP_CYCLE:
				case DAVIS_CONFIG_IMU_LP_WAKEUP:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1:
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2:
					if (handle->info.extInputHasExtraDetectors) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_MICROPHONE:
			switch (paramAddr) {
				case DAVIS_CONFIG_MICROPHONE_RUN:
				case DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_MICROPHONE, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_BIAS: // Also DAVIS_CONFIG_CHIP (starts at address 128).
			if (paramAddr < 128) {
				// BIASING (DAVIS_CONFIG_BIAS).
				if (IS_DAVIS240(handle->info.chipID)) {
					// DAVIS240 uses the old bias generator with 22 branches, and uses all of them.
					if (paramAddr < 22) {
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
					}
				}
				else if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID)
				|| IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
					// All new DAVISes use the new bias generator with 37 branches.
					switch (paramAddr) {
						// Same and shared between all of the above chips.
						case DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL:
						case DAVIS128_CONFIG_BIAS_APSCAS:
						case DAVIS128_CONFIG_BIAS_ADCREFHIGH:
						case DAVIS128_CONFIG_BIAS_ADCREFLOW:
						case DAVIS128_CONFIG_BIAS_LOCALBUFBN:
						case DAVIS128_CONFIG_BIAS_PADFOLLBN:
						case DAVIS128_CONFIG_BIAS_DIFFBN:
						case DAVIS128_CONFIG_BIAS_ONBN:
						case DAVIS128_CONFIG_BIAS_OFFBN:
						case DAVIS128_CONFIG_BIAS_PIXINVBN:
						case DAVIS128_CONFIG_BIAS_PRBP:
						case DAVIS128_CONFIG_BIAS_PRSFBP:
						case DAVIS128_CONFIG_BIAS_REFRBP:
						case DAVIS128_CONFIG_BIAS_READOUTBUFBP:
						case DAVIS128_CONFIG_BIAS_APSROSFBN:
						case DAVIS128_CONFIG_BIAS_ADCCOMPBP:
						case DAVIS128_CONFIG_BIAS_COLSELLOWBN:
						case DAVIS128_CONFIG_BIAS_DACBUFBP:
						case DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVIS128_CONFIG_BIAS_AEPDBN:
						case DAVIS128_CONFIG_BIAS_AEPUXBP:
						case DAVIS128_CONFIG_BIAS_AEPUYBP:
						case DAVIS128_CONFIG_BIAS_IFREFRBN:
						case DAVIS128_CONFIG_BIAS_IFTHRBN:
						case DAVIS128_CONFIG_BIAS_BIASBUFFER:
						case DAVIS128_CONFIG_BIAS_SSP:
						case DAVIS128_CONFIG_BIAS_SSN:
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						case DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE:
							// Only supported by DAVIS346 and DAVIS640 chips.
							if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
								return (spiConfigSend(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						case DAVIS208_CONFIG_BIAS_RESETHIGHPASS:
						case DAVIS208_CONFIG_BIAS_REFSS:
						case DAVIS208_CONFIG_BIAS_REGBIASBP:
						case DAVIS208_CONFIG_BIAS_REFSSBN:
							// Only supported by DAVIS208 chips.
							if (IS_DAVIS208(handle->info.chipID)) {
								return (spiConfigSend(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						default:
							return (false);
							break;
					}
				}
				else if (IS_DAVISRGB(handle->info.chipID)) {
					// DAVISRGB also uses the 37 branches bias generator, with different values.
					switch (paramAddr) {
						case DAVISRGB_CONFIG_BIAS_APSCAS:
						case DAVISRGB_CONFIG_BIAS_OVG1LO:
						case DAVISRGB_CONFIG_BIAS_OVG2LO:
						case DAVISRGB_CONFIG_BIAS_TX2OVG2HI:
						case DAVISRGB_CONFIG_BIAS_GND07:
						case DAVISRGB_CONFIG_BIAS_ADCTESTVOLTAGE:
						case DAVISRGB_CONFIG_BIAS_ADCREFHIGH:
						case DAVISRGB_CONFIG_BIAS_ADCREFLOW:
						case DAVISRGB_CONFIG_BIAS_IFREFRBN:
						case DAVISRGB_CONFIG_BIAS_IFTHRBN:
						case DAVISRGB_CONFIG_BIAS_LOCALBUFBN:
						case DAVISRGB_CONFIG_BIAS_PADFOLLBN:
						case DAVISRGB_CONFIG_BIAS_PIXINVBN:
						case DAVISRGB_CONFIG_BIAS_DIFFBN:
						case DAVISRGB_CONFIG_BIAS_ONBN:
						case DAVISRGB_CONFIG_BIAS_OFFBN:
						case DAVISRGB_CONFIG_BIAS_PRBP:
						case DAVISRGB_CONFIG_BIAS_PRSFBP:
						case DAVISRGB_CONFIG_BIAS_REFRBP:
						case DAVISRGB_CONFIG_BIAS_ARRAYBIASBUFFERBN:
						case DAVISRGB_CONFIG_BIAS_ARRAYLOGICBUFFERBN:
						case DAVISRGB_CONFIG_BIAS_FALLTIMEBN:
						case DAVISRGB_CONFIG_BIAS_RISETIMEBP:
						case DAVISRGB_CONFIG_BIAS_READOUTBUFBP:
						case DAVISRGB_CONFIG_BIAS_APSROSFBN:
						case DAVISRGB_CONFIG_BIAS_ADCCOMPBP:
						case DAVISRGB_CONFIG_BIAS_DACBUFBP:
						case DAVISRGB_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVISRGB_CONFIG_BIAS_AEPDBN:
						case DAVISRGB_CONFIG_BIAS_AEPUXBP:
						case DAVISRGB_CONFIG_BIAS_AEPUYBP:
						case DAVISRGB_CONFIG_BIAS_BIASBUFFER:
						case DAVISRGB_CONFIG_BIAS_SSP:
						case DAVISRGB_CONFIG_BIAS_SSN:
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						default:
							return (false);
							break;
					}
				}
			}
			else {
				// CHIP CONFIGURATION (DAVIS_CONFIG_CHIP).
				switch (paramAddr) {
					// Chip configuration common to all chips.
					case DAVIS128_CONFIG_CHIP_DIGITALMUX0:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX1:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX2:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX3:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX0:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX1:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX2:
					case DAVIS128_CONFIG_CHIP_BIASMUX0:
					case DAVIS128_CONFIG_CHIP_RESETCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_RESETTESTPIXEL:
					case DAVIS128_CONFIG_CHIP_AERNAROW:
					case DAVIS128_CONFIG_CHIP_USEAOUT:
						return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL:
						// Only supported by DAVIS240 A/B chips.
						if (IS_DAVIS240A(handle->info.chipID) || IS_DAVIS240B(handle->info.chipID)) {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							// Keep in sync with APS module GlobalShutter parameter.
							if (!spiConfigSend(&state->usbState, DAVIS_CONFIG_APS,
							DAVIS_CONFIG_APS_GLOBAL_SHUTTER, param)) {
								return (false);
							}

							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						// Only supported by the new DAVIS chips.
						if (IS_DAVIS128(
							handle->info.chipID) || IS_DAVIS208(handle->info.chipID) || IS_DAVIS346(handle->info.chipID)
							|| IS_DAVIS640(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS346_CONFIG_CHIP_TESTADC:
						// Only supported by some of the new DAVIS chips.
						if (IS_DAVIS346(
							handle->info.chipID) || IS_DAVIS640(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVISRGB_CONFIG_CHIP_ADJUSTOVG1LO: // Also DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG.
					case DAVISRGB_CONFIG_CHIP_ADJUSTOVG2LO: // Also DAVIS208_CONFIG_CHIP_SELECTBIASREFSS.
					case DAVISRGB_CONFIG_CHIP_ADJUSTTX2OVG2HI: // Also DAVIS208_CONFIG_CHIP_SELECTSENSE.
						// Only supported by DAVIS208 and DAVISRGB.
						if (IS_DAVIS208(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS208_CONFIG_CHIP_SELECTPOSFB:
					case DAVIS208_CONFIG_CHIP_SELECTHIGHPASS:
						// Only supported by DAVIS208.
						if (IS_DAVIS208(handle->info.chipID)) {
							return (spiConfigSend(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					default:
						return (false);
						break;
				}
			}

			return (false);
			break;

		case DAVIS_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DAVIS_CONFIG_USB:
			switch (paramAddr) {
				case DAVIS_CONFIG_USB_RUN:
				case DAVIS_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigSend(&state->usbState, DAVIS_CONFIG_USB, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool davisCommonConfigGet(davisHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	davisState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
					*param = usbGetTransfersNumber(&state->usbState);
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					*param = usbGetTransfersSize(&state->usbState);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
					*param = U32T(atomic_load(&state->dataExchangeBufferSize));
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
					*param = atomic_load(&state->dataExchangeBlocking);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
					*param = atomic_load(&state->dataExchangeStartProducers);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
					*param = atomic_load(&state->dataExchangeStopProducers);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_PACKETS:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE:
					*param = U32T(atomic_load(&state->maxPacketContainerPacketSize));
					break;

				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL:
					*param = U32T(atomic_load(&state->maxPacketContainerInterval));
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					*param = atomic_load(&state->deviceLogLevel);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_MUX:
			switch (paramAddr) {
				case DAVIS_CONFIG_MUX_RUN:
				case DAVIS_CONFIG_MUX_TIMESTAMP_RUN:
				case DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_DVS:
			switch (paramAddr) {
				case DAVIS_CONFIG_DVS_SIZE_COLUMNS:
				case DAVIS_CONFIG_DVS_SIZE_ROWS:
				case DAVIS_CONFIG_DVS_ORIENTATION_INFO:
				case DAVIS_CONFIG_DVS_RUN:
				case DAVIS_CONFIG_DVS_ACK_DELAY_ROW:
				case DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN:
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
				case DAVIS_CONFIG_DVS_HAS_PIXEL_FILTER:
				case DAVIS_CONFIG_DVS_HAS_BACKGROUND_ACTIVITY_FILTER:
				case DAVIS_CONFIG_DVS_HAS_TEST_EVENT_GENERATOR:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
					if (handle->info.dvsHasPixelFilter) {
						if (handle->state.dvsInvertXY) {
							// Convert to column if X/Y inverted.
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, U8T(paramAddr + 1), param));
						}
						else {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
						}
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						if (handle->state.dvsInvertXY) {
							// Convert to row if X/Y inverted.
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, U8T(paramAddr - 1), param));
						}
						else {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
						}
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_DELTAT:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE:
					if (handle->info.dvsHasTestEventGenerator) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_APS:
			switch (paramAddr) {
				case DAVIS_CONFIG_APS_SIZE_COLUMNS:
				case DAVIS_CONFIG_APS_SIZE_ROWS:
				case DAVIS_CONFIG_APS_ORIENTATION_INFO:
				case DAVIS_CONFIG_APS_COLOR_FILTER:
				case DAVIS_CONFIG_APS_RUN:
				case DAVIS_CONFIG_APS_RESET_READ:
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_ROW_SETTLE:
				case DAVIS_CONFIG_APS_HAS_GLOBAL_SHUTTER:
				case DAVIS_CONFIG_APS_HAS_QUAD_ROI:
				case DAVIS_CONFIG_APS_HAS_EXTERNAL_ADC:
				case DAVIS_CONFIG_APS_HAS_INTERNAL_ADC:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_0:
				case DAVIS_CONFIG_APS_END_COLUMN_0:
					if (state->apsInvertXY) {
						// Convert to row if X/Y inverted.
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, U8T(paramAddr + 1), param));
					}
					else {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					break;

				case DAVIS_CONFIG_APS_START_ROW_0:
				case DAVIS_CONFIG_APS_END_ROW_0:
					if (state->apsInvertXY) {
						// Convert to column if X/Y inverted.
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, U8T(paramAddr - 1), param));
					}
					else {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					break;

				case DAVIS_CONFIG_APS_RESET_SETTLE:
				case DAVIS_CONFIG_APS_COLUMN_SETTLE:
				case DAVIS_CONFIG_APS_NULL_SETTLE:
					// Not supported on DAVIS RGB APS state machine.
					if (!IS_DAVISRGB(handle->info.chipID)) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Use stored value, no need to call out to USB for this one.
					*param = state->apsExposureLastSetValue;
					break;

				case DAVIS_CONFIG_APS_FRAME_DELAY: {
					// Exposure and Frame Delay are in µs, must be converted from native FPGA cycles
					// by dividing with ADC clock value.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, &cyclesValue)) {
						return (false);
					}

					*param = cyclesValue / U16T(handle->info.adcClock);

					return (true);
					break;
				}

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_1:
				case DAVIS_CONFIG_APS_END_COLUMN_1:
				case DAVIS_CONFIG_APS_START_COLUMN_2:
				case DAVIS_CONFIG_APS_END_COLUMN_2:
				case DAVIS_CONFIG_APS_START_COLUMN_3:
				case DAVIS_CONFIG_APS_END_COLUMN_3:
				case DAVIS_CONFIG_APS_START_ROW_1:
				case DAVIS_CONFIG_APS_END_ROW_1:
				case DAVIS_CONFIG_APS_START_ROW_2:
				case DAVIS_CONFIG_APS_END_ROW_2:
				case DAVIS_CONFIG_APS_START_ROW_3:
				case DAVIS_CONFIG_APS_END_ROW_3:
					// TODO: no support on host-side for QuadROI and multi-frame decoding.
					return (false);
					break;

				case DAVIS_CONFIG_APS_USE_INTERNAL_ADC:
				case DAVIS_CONFIG_APS_SAMPLE_ENABLE:
				case DAVIS_CONFIG_APS_SAMPLE_SETTLE:
				case DAVIS_CONFIG_APS_RAMP_RESET:
				case DAVIS_CONFIG_APS_RAMP_SHORT_RESET:
				case DAVIS_CONFIG_APS_ADC_TEST_MODE:
					if (handle->info.apsHasInternalADC) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVISRGB_CONFIG_APS_TRANSFER:
				case DAVISRGB_CONFIG_APS_RSFDSETTLE:
				case DAVISRGB_CONFIG_APS_GSPDRESET:
				case DAVISRGB_CONFIG_APS_GSRESETFALL:
				case DAVISRGB_CONFIG_APS_GSTXFALL:
				case DAVISRGB_CONFIG_APS_GSFDRESET:
					// Support for DAVISRGB extra timing parameters.
					if (IS_DAVISRGB(handle->info.chipID)) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SNAPSHOT:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DAVIS_CONFIG_APS_AUTOEXPOSURE:
					*param = atomic_load(&state->apsAutoExposureEnabled);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN:
				case DAVIS_CONFIG_IMU_TEMP_STANDBY:
				case DAVIS_CONFIG_IMU_ACCEL_STANDBY:
				case DAVIS_CONFIG_IMU_GYRO_STANDBY:
				case DAVIS_CONFIG_IMU_LP_CYCLE:
				case DAVIS_CONFIG_IMU_LP_WAKEUP:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_HAS_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_HAS_EXTRA_DETECTORS:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1:
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2:
					if (handle->info.extInputHasExtraDetectors) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_MICROPHONE:
			switch (paramAddr) {
				case DAVIS_CONFIG_MICROPHONE_RUN:
				case DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_MICROPHONE, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_BIAS: // Also DAVIS_CONFIG_CHIP (starts at address 128).
			if (paramAddr < 128) {
				// BIASING (DAVIS_CONFIG_BIAS).
				if (IS_DAVIS240(handle->info.chipID)) {
					// DAVIS240 uses the old bias generator with 22 branches, and uses all of them.
					if (paramAddr < 22) {
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
					}
				}
				else if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID)
				|| IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
					// All new DAVISes use the new bias generator with 37 branches.
					switch (paramAddr) {
						// Same and shared between all of the above chips.
						case DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL:
						case DAVIS128_CONFIG_BIAS_APSCAS:
						case DAVIS128_CONFIG_BIAS_ADCREFHIGH:
						case DAVIS128_CONFIG_BIAS_ADCREFLOW:
						case DAVIS128_CONFIG_BIAS_LOCALBUFBN:
						case DAVIS128_CONFIG_BIAS_PADFOLLBN:
						case DAVIS128_CONFIG_BIAS_DIFFBN:
						case DAVIS128_CONFIG_BIAS_ONBN:
						case DAVIS128_CONFIG_BIAS_OFFBN:
						case DAVIS128_CONFIG_BIAS_PIXINVBN:
						case DAVIS128_CONFIG_BIAS_PRBP:
						case DAVIS128_CONFIG_BIAS_PRSFBP:
						case DAVIS128_CONFIG_BIAS_REFRBP:
						case DAVIS128_CONFIG_BIAS_READOUTBUFBP:
						case DAVIS128_CONFIG_BIAS_APSROSFBN:
						case DAVIS128_CONFIG_BIAS_ADCCOMPBP:
						case DAVIS128_CONFIG_BIAS_COLSELLOWBN:
						case DAVIS128_CONFIG_BIAS_DACBUFBP:
						case DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVIS128_CONFIG_BIAS_AEPDBN:
						case DAVIS128_CONFIG_BIAS_AEPUXBP:
						case DAVIS128_CONFIG_BIAS_AEPUYBP:
						case DAVIS128_CONFIG_BIAS_IFREFRBN:
						case DAVIS128_CONFIG_BIAS_IFTHRBN:
						case DAVIS128_CONFIG_BIAS_BIASBUFFER:
						case DAVIS128_CONFIG_BIAS_SSP:
						case DAVIS128_CONFIG_BIAS_SSN:
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						case DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE:
							// Only supported by DAVIS346 and DAVIS640 chips.
							if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
								return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						case DAVIS208_CONFIG_BIAS_RESETHIGHPASS:
						case DAVIS208_CONFIG_BIAS_REFSS:
						case DAVIS208_CONFIG_BIAS_REGBIASBP:
						case DAVIS208_CONFIG_BIAS_REFSSBN:
							// Only supported by DAVIS208 chips.
							if (IS_DAVIS208(handle->info.chipID)) {
								return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						default:
							return (false);
							break;
					}
				}
				else if (IS_DAVISRGB(handle->info.chipID)) {
					// DAVISRGB also uses the 37 branches bias generator, with different values.
					switch (paramAddr) {
						case DAVISRGB_CONFIG_BIAS_APSCAS:
						case DAVISRGB_CONFIG_BIAS_OVG1LO:
						case DAVISRGB_CONFIG_BIAS_OVG2LO:
						case DAVISRGB_CONFIG_BIAS_TX2OVG2HI:
						case DAVISRGB_CONFIG_BIAS_GND07:
						case DAVISRGB_CONFIG_BIAS_ADCTESTVOLTAGE:
						case DAVISRGB_CONFIG_BIAS_ADCREFHIGH:
						case DAVISRGB_CONFIG_BIAS_ADCREFLOW:
						case DAVISRGB_CONFIG_BIAS_IFREFRBN:
						case DAVISRGB_CONFIG_BIAS_IFTHRBN:
						case DAVISRGB_CONFIG_BIAS_LOCALBUFBN:
						case DAVISRGB_CONFIG_BIAS_PADFOLLBN:
						case DAVISRGB_CONFIG_BIAS_PIXINVBN:
						case DAVISRGB_CONFIG_BIAS_DIFFBN:
						case DAVISRGB_CONFIG_BIAS_ONBN:
						case DAVISRGB_CONFIG_BIAS_OFFBN:
						case DAVISRGB_CONFIG_BIAS_PRBP:
						case DAVISRGB_CONFIG_BIAS_PRSFBP:
						case DAVISRGB_CONFIG_BIAS_REFRBP:
						case DAVISRGB_CONFIG_BIAS_ARRAYBIASBUFFERBN:
						case DAVISRGB_CONFIG_BIAS_ARRAYLOGICBUFFERBN:
						case DAVISRGB_CONFIG_BIAS_FALLTIMEBN:
						case DAVISRGB_CONFIG_BIAS_RISETIMEBP:
						case DAVISRGB_CONFIG_BIAS_READOUTBUFBP:
						case DAVISRGB_CONFIG_BIAS_APSROSFBN:
						case DAVISRGB_CONFIG_BIAS_ADCCOMPBP:
						case DAVISRGB_CONFIG_BIAS_DACBUFBP:
						case DAVISRGB_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVISRGB_CONFIG_BIAS_AEPDBN:
						case DAVISRGB_CONFIG_BIAS_AEPUXBP:
						case DAVISRGB_CONFIG_BIAS_AEPUYBP:
						case DAVISRGB_CONFIG_BIAS_BIASBUFFER:
						case DAVISRGB_CONFIG_BIAS_SSP:
						case DAVISRGB_CONFIG_BIAS_SSN:
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						default:
							return (false);
							break;
					}
				}
			}
			else {
				// CHIP CONFIGURATION (DAVIS_CONFIG_CHIP).
				switch (paramAddr) {
					// Chip configuration common to all chips.
					case DAVIS128_CONFIG_CHIP_DIGITALMUX0:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX1:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX2:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX3:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX0:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX1:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX2:
					case DAVIS128_CONFIG_CHIP_BIASMUX0:
					case DAVIS128_CONFIG_CHIP_RESETCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_RESETTESTPIXEL:
					case DAVIS128_CONFIG_CHIP_AERNAROW:
					case DAVIS128_CONFIG_CHIP_USEAOUT:
						return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL:
						// Only supported by DAVIS240 A/B chips.
						if (IS_DAVIS240A(handle->info.chipID) || IS_DAVIS240B(handle->info.chipID)) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						// Only supported by the new DAVIS chips.
						if (IS_DAVIS128(
							handle->info.chipID) || IS_DAVIS208(handle->info.chipID) || IS_DAVIS346(handle->info.chipID)
							|| IS_DAVIS640(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS346_CONFIG_CHIP_TESTADC:
						// Only supported by some of the new DAVIS chips.
						if (IS_DAVIS346(
							handle->info.chipID) || IS_DAVIS640(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVISRGB_CONFIG_CHIP_ADJUSTOVG1LO: // Also DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG.
					case DAVISRGB_CONFIG_CHIP_ADJUSTOVG2LO: // Also DAVIS208_CONFIG_CHIP_SELECTBIASREFSS.
					case DAVISRGB_CONFIG_CHIP_ADJUSTTX2OVG2HI: // Also DAVIS208_CONFIG_CHIP_SELECTSENSE.
						// Only supported by DAVIS208 and DAVISRGB.
						if (IS_DAVIS208(handle->info.chipID) || IS_DAVISRGB(handle->info.chipID)) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS208_CONFIG_CHIP_SELECTPOSFB:
					case DAVIS208_CONFIG_CHIP_SELECTHIGHPASS:
						// Only supported by DAVIS208.
						if (IS_DAVIS208(handle->info.chipID)) {
							return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					default:
						return (false);
						break;
				}
			}

			return (false);
			break;

		case DAVIS_CONFIG_SYSINFO:
			switch (paramAddr) {
				case DAVIS_CONFIG_SYSINFO_LOGIC_VERSION:
				case DAVIS_CONFIG_SYSINFO_CHIP_IDENTIFIER:
				case DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER:
				case DAVIS_CONFIG_SYSINFO_LOGIC_CLOCK:
				case DAVIS_CONFIG_SYSINFO_ADC_CLOCK:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_SYSINFO, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_USB:
			switch (paramAddr) {
				case DAVIS_CONFIG_USB_RUN:
				case DAVIS_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigReceive(&state->usbState, DAVIS_CONFIG_USB, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool davisCommonDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	davisHandle handle = (davisHandle) cdh;
	davisState state = &handle->state;

	// Store new data available/not available anymore call-backs.
	state->dataNotifyIncrease = dataNotifyIncrease;
	state->dataNotifyDecrease = dataNotifyDecrease;
	state->dataNotifyUserPtr = dataNotifyUserPtr;

	usbSetShutdownCallback(&state->usbState, dataShutdownNotify, dataShutdownUserPtr);

	// Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
	// will then set this correctly.
	state->currentPacketContainerCommitTimestamp = -1;

	// Initialize RingBuffer.
	state->dataExchangeBuffer = ringBufferInit(atomic_load(&state->dataExchangeBufferSize));
	if (state->dataExchangeBuffer == NULL) {
		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPolarityPacket = caerPolarityEventPacketAllocate(DAVIS_POLARITY_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPolarityPacket == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(DAVIS_SPECIAL_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	state->currentFramePacket = caerFrameEventPacketAllocate(DAVIS_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), 0,
		state->apsSizeX, state->apsSizeY, 1);
	if (state->currentFramePacket == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
		return (false);
	}

	// Allocate memory for the current FrameEvents. Use contiguous memory for all ROI FrameEvents.
	size_t eventSize = (sizeof(struct caer_frame_event) - sizeof(uint16_t))
		+ ((size_t) state->apsSizeX * (size_t) state->apsSizeY * APS_ADC_CHANNELS * sizeof(uint16_t));

	state->currentFrameEvent[0] = calloc(APS_ROI_REGIONS_MAX, eventSize);
	if (state->currentFrameEvent[0] == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate ROI frame events.");
		return (false);
	}

	for (size_t i = 1; i < APS_ROI_REGIONS_MAX; i++) {
		// Assign the right memory offset to the pointers into the block that
		// contains all the ROI FrameEvents.
		state->currentFrameEvent[i] = (caerFrameEvent) (((uint8_t*) state->currentFrameEvent[0]) + (i * eventSize));
	}

	state->currentIMU6Packet = caerIMU6EventPacketAllocate(DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentIMU6Packet == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
		return (false);
	}

	state->currentSamplePacket = caerSampleEventPacketAllocate(DAVIS_SAMPLE_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSamplePacket == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate Sample event packet.");
		return (false);
	}

	state->apsCurrentResetFrame = calloc((size_t) (state->apsSizeX * state->apsSizeY * APS_ADC_CHANNELS),
		sizeof(uint16_t));
	if (state->apsCurrentResetFrame == NULL) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS reset frame memory.");
		return (false);
	}

	// Default IMU settings (for event parsing).
	uint32_t param32 = 0;

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, &param32);
	state->imuAccelScale = calculateIMUAccelScale(U8T(param32));
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, &param32);
	state->imuGyroScale = calculateIMUGyroScale(U8T(param32));

	// Disable all ROI regions by setting them to -1.
	for (size_t i = 0; i < APS_ROI_REGIONS_MAX; i++) {
		state->apsROISizeX[i] = state->apsROIPositionX[i] = U16T(state->apsSizeX);
		state->apsROISizeY[i] = state->apsROIPositionY[i] = U16T(state->apsSizeY);
	}

	// Ignore multi-part events (APS and IMU) at startup, so that any initial
	// incomplete event is ignored. The START events reset this as soon as
	// the first one is observed.
	state->apsIgnoreEvents = true;
	state->imuIgnoreEvents = true;

	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, &param32);
	state->apsGlobalShutter = param32;
	spiConfigReceive(&state->usbState, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ, &param32);
	state->apsResetRead = param32;

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 2.
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, true);
		// Do NOT enable additional ExtInput detectors, those are always user controlled.
		// Do NOT enable microphones by default.

		// Enable data transfer only after enabling the data producers, so that the chip
		// has time to start up and we avoid the initial data flood.
		struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 500000000 };
		thrd_sleep(&noDataSleep, NULL);

		davisCommonConfigSet(handle, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, true);
	}

	return (true);
}

bool davisCommonDataStop(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;
	davisState state = &handle->state;

	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		davisCommonConfigSet(handle, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_RUN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false); // Ensure chip turns off.
		davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, false); // Turn off timestamping too.
		davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, false);
	}

	usbDataTransfersStop(&state->usbState);

	// Empty ringbuffer.
	caerEventPacketContainer container;
	while ((container = ringBufferGet(state->dataExchangeBuffer)) != NULL) {
		// Notify data-not-available call-back.
		if (state->dataNotifyDecrease != NULL) {
			state->dataNotifyDecrease(state->dataNotifyUserPtr);
		}

		// Free container, which will free its subordinate packets too.
		caerEventPacketContainerFree(container);
	}

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPolarityPacketPosition = 0;
	state->currentSpecialPacketPosition = 0;
	state->currentFramePacketPosition = 0;
	state->currentIMU6PacketPosition = 0;
	state->currentSamplePacketPosition = 0;

	// Reset private composite events. 'currentFrameEvent' is taken care of in freeAllDataMemory().
	memset(&state->currentIMU6Event, 0, sizeof(struct caer_imu6_event));

	return (true);
}

caerEventPacketContainer davisCommonDataGet(caerDeviceHandle cdh) {
	davisHandle handle = (davisHandle) cdh;
	davisState state = &handle->state;
	caerEventPacketContainer container = NULL;

	retry: container = ringBufferGet(state->dataExchangeBuffer);

	if (container != NULL) {
		// Found an event container, return it and signal this piece of data
		// is no longer available for later acquisition.
		if (state->dataNotifyDecrease != NULL) {
			state->dataNotifyDecrease(state->dataNotifyUserPtr);
		}

		return (container);
	}

	// Didn't find any event container, either report this or retry, depending
	// on blocking setting.
	if (atomic_load_explicit(&state->dataExchangeBlocking, memory_order_relaxed)) {
		// Don't retry right away in a tight loop, back off and wait a little.
		// If no data is available, sleep for a millisecond to avoid wasting resources.
		struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 1000000 };
		if (thrd_sleep(&noDataSleep, NULL) == 0) {
			goto retry;
		}
	}

	// Nothing.
	return (NULL);
}

#define TS_WRAP_ADD 0x8000

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T((U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void initContainerCommitTimestamp(davisState state) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = state->currentTimestamp
			+ I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
	}
}

static void davisEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
	davisHandle handle = vhd;
	davisState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		davisCommonLog(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of two.",
			bytesSent);
		bytesSent &= (size_t) ~0x01;
	}

	for (size_t i = 0; i < bytesSent; i += 2) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentPolarityPacket == NULL) {
			state->currentPolarityPacket = caerPolarityEventPacketAllocate(
			DAVIS_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentPolarityPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}
		else if (state->currentPolarityPacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPolarityPacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerPolarityEventPacket grownPacket = (caerPolarityEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPolarityPacket, state->currentPolarityPacketPosition * 2);
			if (grownPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to grow polarity event packet.");
				return;
			}

			state->currentPolarityPacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(
			DAVIS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}
		else if (state->currentSpecialPacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpecialPacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentSpecialPacket, state->currentSpecialPacketPosition * 2);
			if (grownPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentSpecialPacket = grownPacket;
		}

		if (state->currentFramePacket == NULL) {
			state->currentFramePacket = caerFrameEventPacketAllocate(
			DAVIS_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow, state->apsSizeX,
				state->apsSizeY, 1);
			if (state->currentFramePacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
				return;
			}
		}
		else if (state->currentFramePacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentFramePacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerFrameEventPacket grownPacket = (caerFrameEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentFramePacket, state->currentFramePacketPosition * 2);
			if (grownPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to grow frame event packet.");
				return;
			}

			state->currentFramePacket = grownPacket;
		}

		if (state->currentIMU6Packet == NULL) {
			state->currentIMU6Packet = caerIMU6EventPacketAllocate(
			DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentIMU6Packet == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
				return;
			}
		}
		else if (state->currentIMU6PacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentIMU6Packet)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerIMU6EventPacket grownPacket = (caerIMU6EventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentIMU6Packet, state->currentIMU6PacketPosition * 2);
			if (grownPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to grow IMU6 event packet.");
				return;
			}

			state->currentIMU6Packet = grownPacket;
		}

		if (state->currentSamplePacket == NULL) {
			state->currentSamplePacket = caerSampleEventPacketAllocate(
			DAVIS_SAMPLE_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSamplePacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate Sample event packet.");
				return;
			}
		}
		else if (state->currentSamplePacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSamplePacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSampleEventPacket grownPacket = (caerSampleEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentSamplePacket, state->currentSamplePacketPosition * 2);
			if (grownPacket == NULL) {
				davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to grow Sample event packet.");
				return;
			}

			state->currentSamplePacket = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(*((uint16_t * ) (&buffer[i])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			// Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
			state->lastTimestamp = state->currentTimestamp;
			state->currentTimestamp = state->wrapAdd + (event & 0x7FFF);
			initContainerCommitTimestamp(state);

			// Check monotonicity of timestamps.
			checkStrictMonotonicTimestamp(handle);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							davisCommonLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							state->wrapOverflow = 0;
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;
							state->currentPacketContainerCommitTimestamp = -1;
							initContainerCommitTimestamp(state);

							davisCommonLog(CAER_LOG_INFO, handle, "Timestamp reset event received.");

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets.
							// Async call to not deadlock here.
							spiConfigReceiveAsync(&state->usbState, DAVIS_CONFIG_SYSINFO,
							DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER, &davisTSMasterStatusUpdater, &handle->info);
							break;
						}

						case 2: { // External input (falling edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 3: { // External input (rising edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 4: { // External input (pulse)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 5: { // IMU Start (6 axes)
							davisCommonLog(CAER_LOG_DEBUG, handle, "IMU6 Start event received.");

							state->imuIgnoreEvents = false;
							state->imuCount = 0;

							memset(&state->currentIMU6Event, 0, sizeof(struct caer_imu6_event));

							break;
						}

						case 7: { // IMU End
							davisCommonLog(CAER_LOG_DEBUG, handle, "IMU End event received.");
							if (state->imuIgnoreEvents) {
								break;
							}

							if (state->imuCount == IMU6_COUNT) {
								// Timestamp at event-stream insertion point.
								caerIMU6EventSetTimestamp(&state->currentIMU6Event, state->currentTimestamp);

								caerIMU6EventValidate(&state->currentIMU6Event, state->currentIMU6Packet);

								// IMU6 and APS operate on an internal event and copy that to the actual output
								// packet here, in the END state, for a reason: if a packetContainer, with all its
								// packets, is committed due to hitting any of the triggers that are not TS reset
								// or TS wrap-around related, like number of polarity events, the event in the packet
								// would be left incomplete, and the event in the new packet would be corrupted.
								// We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
								// just deleting that event, but these kinds of commits happen much more often and the
								// possible data loss would be too significant. So instead we keep a private event,
								// fill it, and then only copy it into the packet here in the END state, at which point
								// the whole event is ready and cannot be broken/corrupted in any way anymore.
								caerIMU6Event currentIMU6Event = caerIMU6EventPacketGetEvent(state->currentIMU6Packet,
									state->currentIMU6PacketPosition);
								memcpy(currentIMU6Event, &state->currentIMU6Event, sizeof(struct caer_imu6_event));
								state->currentIMU6PacketPosition++;
							}
							else {
								davisCommonLog(CAER_LOG_INFO, handle,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imuCount);
							}
							break;
						}

						case 8: { // APS Global Shutter Frame Start
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS GS Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = true;
							state->apsResetRead = true;

							initFrame(handle);

							break;
						}

						case 9: { // APS Rolling Shutter Frame Start
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS RS Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = false;
							state->apsResetRead = true;

							initFrame(handle);

							break;
						}

						case 10: { // APS Frame End
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Frame End event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							bool validFrame = true;

							for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
								int32_t checkValue = caerFrameEventGetLengthX(state->currentFrameEvent[0]);

								// Check main reset read against zero if disabled.
								if (j == APS_READOUT_RESET && !state->apsResetRead) {
									checkValue = 0;
								}

								davisCommonLog(CAER_LOG_DEBUG, handle, "APS Frame End: CountX[%zu] is %d.", j,
									state->apsCountX[j]);

								if (state->apsCountX[j] != checkValue) {
									davisCommonLog(CAER_LOG_ERROR, handle,
										"APS Frame End - %zu: wrong column count %d detected, expected %d.", j,
										state->apsCountX[j], checkValue);
									validFrame = false;
								}
							}

							// Write out end of frame timestamp.
							caerFrameEventSetTSEndOfFrame(state->currentFrameEvent[0], state->currentTimestamp);

							// Send APS info event out (as special event).
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_END);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;

							// Validate event and advance frame packet position.
							if (validFrame) {
								caerFrameEventValidate(state->currentFrameEvent[0], state->currentFramePacket);

								// Invert X and Y axes if image from chip is inverted.
								if (state->apsInvertXY) {
									SWAP_VAR(int32_t, state->currentFrameEvent[0]->lengthX,
										state->currentFrameEvent[0]->lengthY);
									SWAP_VAR(int32_t, state->currentFrameEvent[0]->positionX,
										state->currentFrameEvent[0]->positionY);
								}

								// IMU6 and APS operate on an internal event and copy that to the actual output
								// packet here, in the END state, for a reason: if a packetContainer, with all its
								// packets, is committed due to hitting any of the triggers that are not TS reset
								// or TS wrap-around related, like number of polarity events, the event in the packet
								// would be left incomplete, and the event in the new packet would be corrupted.
								// We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
								// just deleting that event, but these kinds of commits happen much more often and the
								// possible data loss would be too significant. So instead we keep a private event,
								// fill it, and then only copy it into the packet here in the END state, at which point
								// the whole event is ready and cannot be broken/corrupted in any way anymore.
								caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
									state->currentFramePacket, state->currentFramePacketPosition);
								memcpy(currentFrameEvent, state->currentFrameEvent[0],
									(sizeof(struct caer_frame_event) - sizeof(uint16_t))
										+ caerFrameEventGetPixelsSize(state->currentFrameEvent[0]));
								state->currentFramePacketPosition++;

								// Automatic exposure control support.
								if (atomic_load_explicit(&state->apsAutoExposureEnabled, memory_order_relaxed)) {
									int32_t newExposureValue = autoExposureCalculate(&state->apsAutoExposureState,
										currentFrameEvent, state->apsExposureFrameValue / U16T(handle->info.adcClock),
										state->apsExposureLastSetValue);

									if (newExposureValue >= 0) {
										// Update exposure value. Done in main thread to avoid deadlock inside callback.
										davisCommonLog(CAER_LOG_DEBUG, handle,
											"Automatic exposure control set exposure to %" PRIu32 " µs.",
											newExposureValue);

										state->apsExposureLastSetValue = U32T(newExposureValue);
										spiConfigSendAsync(&state->usbState, DAVIS_CONFIG_APS,
										DAVIS_CONFIG_APS_EXPOSURE, U32T(newExposureValue * U16T(handle->info.adcClock)),
										NULL, NULL);
									}
								}
							}

							break;
						}

						case 11: { // APS Reset Column Start
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Reset Column Start event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							state->apsCurrentReadoutType = APS_READOUT_RESET;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							state->apsRGBPixelOffsetDirection = 0;
							state->apsRGBPixelOffset = 1; // RGB support, first pixel of row always even.

							// The first Reset Column Read Start is also the start
							// of the exposure for the RS.
							if (!state->apsGlobalShutter && state->apsCountX[APS_READOUT_RESET] == 0) {
								caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0],
									state->currentTimestamp);

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentSpecialPacket, state->currentSpecialPacketPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
								caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
								state->currentSpecialPacketPosition++;
							}

							break;
						}

						case 12: { // APS Signal Column Start
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Signal Column Start event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							state->apsCurrentReadoutType = APS_READOUT_SIGNAL;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							state->apsRGBPixelOffsetDirection = 0;
							state->apsRGBPixelOffset = 1; // RGB support, first pixel of row always even.

							// The first Signal Column Read Start is also always the end
							// of the exposure time, for both RS and GS.
							if (state->apsCountX[APS_READOUT_SIGNAL] == 0) {
								caerFrameEventSetTSEndOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentSpecialPacket, state->currentSpecialPacketPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_END);
								caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
								state->currentSpecialPacketPosition++;
							}

							break;
						}

						case 13: { // APS Column End
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Column End event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Column End: CountX[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountX[state->apsCurrentReadoutType]);
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS Column End: CountY[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType]);

							if (state->apsCountY[state->apsCurrentReadoutType]
								!= caerFrameEventGetLengthY(state->currentFrameEvent[0])) {
								davisCommonLog(CAER_LOG_ERROR, handle,
									"APS Column End - %d: wrong row count %d detected, expected %d.",
									state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType],
									caerFrameEventGetLengthY(state->currentFrameEvent[0]));
							}

							state->apsCountX[state->apsCurrentReadoutType]++;

							// The last Reset Column Read End is also the start
							// of the exposure for the GS.
							if (state->apsGlobalShutter && state->apsCurrentReadoutType == APS_READOUT_RESET
								&& state->apsCountX[APS_READOUT_RESET]
									== caerFrameEventGetLengthX(state->currentFrameEvent[0])) {
								caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0],
									state->currentTimestamp);

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentSpecialPacket, state->currentSpecialPacketPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
								caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
								state->currentSpecialPacketPosition++;
							}

							break;
						}

						case 14: { // APS Global Shutter Frame Start with no Reset Read
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS GS NORST Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = true;
							state->apsResetRead = false;

							initFrame(handle);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

							// No APS info event is sent out (as special event). Only one event
							// per type can be sent out per cycle, and initFrame() already does
							// that and sets APS_FRAME_START.

							break;
						}

						case 15: { // APS Rolling Shutter Frame Start with no Reset Read
							davisCommonLog(CAER_LOG_DEBUG, handle, "APS RS NORST Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = false;
							state->apsResetRead = false;

							initFrame(handle);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							caerFrameEventSetTSStartOfExposure(state->currentFrameEvent[0], state->currentTimestamp);

							// No APS info event is sent out (as special event). Only one event
							// per type can be sent out per cycle, and initFrame() already does
							// that and sets APS_FRAME_START.

							break;
						}

						case 16:
						case 17:
						case 18:
						case 19:
						case 20:
						case 21:
						case 22:
						case 23:
						case 24:
						case 25:
						case 26:
						case 27:
						case 28:
						case 29:
						case 30:
						case 31: {
							davisCommonLog(CAER_LOG_DEBUG, handle, "IMU Scale Config event (%" PRIu16 ") received.",
								data);
							if (state->imuIgnoreEvents) {
								break;
							}

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imuAccelScale = calculateIMUAccelScale((data >> 2) & 0x03);
							state->imuGyroScale = calculateIMUGyroScale(data & 0x03);

							// At this point the IMU event count should be zero (reset by start).
							if (state->imuCount != 0) {
								davisCommonLog(CAER_LOG_INFO, handle,
									"IMU Scale Config: previous IMU start event missed, attempting recovery.");
							}

							// Increase IMU count by one, to a total of one (0+1=1).
							// This way we can recover from the above error of missing start, and we can
							// later discover if the IMU Scale Config event actually arrived itself.
							state->imuCount = 1;

							break;
						}

						case 32: {
							// Next Misc8 APS ROI Size events will refer to ROI region 0.
							// 0/1 used to distinguish between X and Y sizes.
							state->apsROIUpdate = (0 << 2);
							state->apsROISizeX[0] = state->apsROIPositionX[0] = U16T(state->apsSizeX);
							state->apsROISizeY[0] = state->apsROIPositionY[0] = U16T(state->apsSizeY);
							break;
						}

						case 33: {
							// Next Misc8 APS ROI Size events will refer to ROI region 1.
							// 2/3 used to distinguish between X and Y sizes.
							state->apsROIUpdate = (1 << 2);
							state->apsROISizeX[1] = state->apsROIPositionX[1] = U16T(state->apsSizeX);
							state->apsROISizeY[1] = state->apsROIPositionY[1] = U16T(state->apsSizeY);
							break;
						}

						case 34: {
							// Next Misc8 APS ROI Size events will refer to ROI region 2.
							// 4/5 used to distinguish between X and Y sizes.
							state->apsROIUpdate = (2 << 2);
							state->apsROISizeX[2] = state->apsROIPositionX[2] = U16T(state->apsSizeX);
							state->apsROISizeY[2] = state->apsROIPositionY[2] = U16T(state->apsSizeY);
							break;
						}

						case 35: {
							// Next Misc8 APS ROI Size events will refer to ROI region 3.
							// 6/7 used to distinguish between X and Y sizes.
							state->apsROIUpdate = (3 << 2);
							state->apsROISizeX[3] = state->apsROIPositionX[3] = U16T(state->apsSizeX);
							state->apsROISizeY[3] = state->apsROIPositionY[3] = U16T(state->apsSizeY);
							break;
						}

						case 36: { // External input 1 (falling edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 1 (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 37: { // External input 1 (rising edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 1 (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 38: { // External input 1 (pulse)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 1 (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 39: { // External input 2 (falling edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 2 (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 40: { // External input 2 (rising edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 2 (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 41: { // External input 2 (pulse)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External input 2 (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 42: { // External generator (falling edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External generator (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 43: { // External generator (rising edge)
							davisCommonLog(CAER_LOG_DEBUG, handle, "External generator (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 48: { // Exposure information.
							// Reset counter and value.
							state->apsExposureFrameUpdate = 0;
							state->apsExposureFrameValue = 0;
							break;
						}

						default:
							davisCommonLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.",
								data);
							break;
					}
					break;

				case 1: // Y address
					// Check range conformity.
					if (data >= state->dvsSizeY) {
						davisCommonLog(CAER_LOG_ALERT, handle, "DVS: Y address out of range (0-%d): %" PRIu16 ".",
							state->dvsSizeY - 1, data);
						break; // Skip invalid Y address (don't update lastY).
					}

					if (state->dvsGotY) {
						caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
							state->currentSpecialPacket, state->currentSpecialPacketPosition);

						// Timestamp at event-stream insertion point.
						caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
						caerSpecialEventSetType(currentSpecialEvent, DVS_ROW_ONLY);
						caerSpecialEventSetData(currentSpecialEvent, state->dvsLastY);
						caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
						state->currentSpecialPacketPosition++;

						davisCommonLog(CAER_LOG_DEBUG, handle,
							"DVS: row-only event received for address Y=%" PRIu16 ".", state->dvsLastY);
					}

					state->dvsLastY = data;
					state->dvsGotY = true;

					break;

				case 2: // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= state->dvsSizeX) {
						davisCommonLog(CAER_LOG_ALERT, handle, "DVS: X address out of range (0-%d): %" PRIu16 ".",
							state->dvsSizeX - 1, data);
						break; // Skip invalid event.
					}

					// Invert polarity for PixelParade high gain pixels (DavisSense), because of
					// negative gain from pre-amplifier.
					uint8_t polarity = ((IS_DAVIS208(handle->info.chipID)) && (data < 192)) ? U8T(~code) : (code);

					caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
						state->currentPolarityPacket, state->currentPolarityPacketPosition);

					// Timestamp at event-stream insertion point.
					caerPolarityEventSetTimestamp(currentPolarityEvent, state->currentTimestamp);
					caerPolarityEventSetPolarity(currentPolarityEvent, (polarity & 0x01));
					if (state->dvsInvertXY) {
						// Flip Y address to conform to CG format.
						caerPolarityEventSetY(currentPolarityEvent, U16T((state->dvsSizeX - 1) - data));
						caerPolarityEventSetX(currentPolarityEvent, state->dvsLastY);
					}
					else {
						// Flip Y address to conform to CG format.
						caerPolarityEventSetY(currentPolarityEvent, U16T((state->dvsSizeY - 1) - state->dvsLastY));
						caerPolarityEventSetX(currentPolarityEvent, data);
					}
					caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);
					state->currentPolarityPacketPosition++;

					state->dvsGotY = false;

					break;
				}

				case 4: {
					if (state->apsIgnoreEvents) {
						break;
					}

					// Let's check that apsCountX is not above the maximum. This could happen
					// if the maximum is a smaller number that comes from ROI, while we're still
					// reading out a frame with a bigger, old size.
					if (state->apsCountX[state->apsCurrentReadoutType]
						>= caerFrameEventGetLengthX(state->currentFrameEvent[0])) {
						davisCommonLog(CAER_LOG_DEBUG, handle,
							"APS ADC sample: column count is at maximum, discarding further samples.");
						break;
					}

					// Let's check that apsCountY is not above the maximum. This could happen
					// if start/end of column events are discarded (no wait on transfer stall).
					if (state->apsCountY[state->apsCurrentReadoutType]
						>= caerFrameEventGetLengthY(state->currentFrameEvent[0])) {
						davisCommonLog(CAER_LOG_DEBUG, handle,
							"APS ADC sample: row count is at maximum, discarding further samples.");
						break;
					}

					// If reset read, we store the values in a local array. If signal read, we
					// store the final pixel value directly in the output frame event. We already
					// do the subtraction between reset and signal here, to avoid carrying that
					// around all the time and consuming memory. This way we can also only take
					// infrequent reset reads and re-use them for multiple frames, which can heavily
					// reduce traffic, and should not impact image quality heavily, at least in GS.
					uint16_t xPos =
						(state->apsFlipX) ?
							(U16T(
								caerFrameEventGetLengthX(state->currentFrameEvent[0]) - 1
									- state->apsCountX[state->apsCurrentReadoutType])) :
							(U16T(state->apsCountX[state->apsCurrentReadoutType]));
					uint16_t yPos =
						(state->apsFlipY) ?
							(U16T(
								caerFrameEventGetLengthY(state->currentFrameEvent[0]) - 1
									- state->apsCountY[state->apsCurrentReadoutType])) :
							(U16T(state->apsCountY[state->apsCurrentReadoutType]));

					if (IS_DAVISRGB(handle->info.chipID)) {
						yPos = U16T(yPos + state->apsRGBPixelOffset);
					}

					int32_t stride = 0;

					if (state->apsInvertXY) {
						SWAP_VAR(uint16_t, xPos, yPos);

						stride = caerFrameEventGetLengthY(state->currentFrameEvent[0]);

						// Flip Y address to conform to CG format.
						yPos = U16T(caerFrameEventGetLengthX(state->currentFrameEvent[0]) - 1 - yPos);
					}
					else {
						stride = caerFrameEventGetLengthX(state->currentFrameEvent[0]);

						// Flip Y address to conform to CG format.
						yPos = U16T(caerFrameEventGetLengthY(state->currentFrameEvent[0]) - 1 - yPos);
					}

					size_t pixelPosition = (size_t) (yPos * stride) + xPos;

					// DAVIS240 has a reduced dynamic range due to external
					// ADC high/low ref resistors not having optimal values.
					// To fix this multiply by 1.95 to 2.15, so we choose to
					// just shift by one (multiply by 2.00) for efficiency.
					if (IS_DAVIS240(handle->info.chipID)) {
						data = U16T(data << 1);
					}

					if ((state->apsCurrentReadoutType == APS_READOUT_RESET
						&& !(IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter))
						|| (state->apsCurrentReadoutType == APS_READOUT_SIGNAL
							&& (IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter))) {
						state->apsCurrentResetFrame[pixelPosition] = data;
					}
					else {
						uint16_t resetValue = 0;
						uint16_t signalValue = 0;

						if (IS_DAVISRGB(handle->info.chipID) && state->apsGlobalShutter) {
							// DAVIS RGB GS has inverted samples, signal read comes first
							// and was stored above inside state->apsCurrentResetFrame.
							resetValue = data;
							signalValue = state->apsCurrentResetFrame[pixelPosition];
						}
						else {
							resetValue = state->apsCurrentResetFrame[pixelPosition];
							signalValue = data;
						}

						int32_t pixelValue = 0;

#if defined(APS_DEBUG_FRAME) && APS_DEBUG_FRAME == 1
						pixelValue = resetValue;

						// Check for overflow.
						pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
#elif defined(APS_DEBUG_FRAME) && APS_DEBUG_FRAME == 2
						pixelValue = signalValue;

						// Check for overflow.
						pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
#else
						if (resetValue < 512 || signalValue == 0) {
							// If the signal value is 0, that is only possible if the camera
							// has seen tons of light. In that case, the photo-diode current
							// may be greater than the reset current, and the reset value
							// never goes back up fully, which results in black spots where
							// there is too much light. This confuses algorithms, so we filter
							// this out here by setting the pixel to white in that case.
							// Another effect of the same thing is the reset value not going
							// back up to a decent value, so we also filter that out here.
							pixelValue = 1023;
						}
						else {
							// Do CDS.
							pixelValue = resetValue - signalValue;

							// Check for underflow.
							pixelValue = (pixelValue < 0) ? (0) : (pixelValue);

							// Check for overflow.
							pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
						}
#endif

						// Normalize the ADC value to 16bit generic depth. This depends on ADC used.
						pixelValue = pixelValue << (16 - APS_ADC_DEPTH);

						caerFrameEventGetPixelArrayUnsafe(state->currentFrameEvent[0])[pixelPosition] = htole16(
							U16T(pixelValue));
					}

					davisCommonLog(CAER_LOG_DEBUG, handle,
						"APS ADC Sample: column=%" PRIu16 ", row=%" PRIu16 ", xPos=%" PRIu16 ", yPos=%" PRIu16 ", data=%" PRIu16 ".",
						state->apsCountX[state->apsCurrentReadoutType], state->apsCountY[state->apsCurrentReadoutType],
						xPos, yPos, data);

					state->apsCountY[state->apsCurrentReadoutType]++;

					// RGB support: first 320 pixels are even, then odd.
					if (IS_DAVISRGB(handle->info.chipID)) {
						if (state->apsRGBPixelOffsetDirection == 0) { // Increasing
							state->apsRGBPixelOffset++;

							if (state->apsRGBPixelOffset == 321) {
								// Switch to decreasing after last even pixel.
								state->apsRGBPixelOffsetDirection = 1;
								state->apsRGBPixelOffset = 318;
							}
						}
						else { // Decreasing
							state->apsRGBPixelOffset = I16T(state->apsRGBPixelOffset - 3);
						}
					}

					break;
				}

				case 5: {
					// Misc 8bit data.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imuIgnoreEvents) {
								break;
							}

							// Detect missing IMU end events.
							if (state->imuCount >= IMU6_COUNT) {
								davisCommonLog(CAER_LOG_INFO, handle,
									"IMU data: IMU samples count is at maximum, discarding further samples.");
								break;
							}

							// IMU data event.
							switch (state->imuCount) {
								case 0:
									davisCommonLog(CAER_LOG_ERROR, handle,
										"IMU data: missing IMU Scale Config event. Parsing of IMU events will still be attempted, but be aware that Accel/Gyro scale conversions may be inaccurate.");
									state->imuCount = 1;
									// Fall through to next case, as if imuCount was equal to 1.

								case 1:
								case 3:
								case 5:
								case 7:
								case 9:
								case 11:
								case 13:
									state->imuTmpData = misc8Data;
									break;

								case 2: {
									int16_t accelX = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipX) {
										accelX = I16T(-accelX);
									}
									caerIMU6EventSetAccelX(&state->currentIMU6Event, accelX / state->imuAccelScale);
									break;
								}

								case 4: {
									int16_t accelY = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipY) {
										accelY = I16T(-accelY);
									}
									caerIMU6EventSetAccelY(&state->currentIMU6Event, accelY / state->imuAccelScale);
									break;
								}

								case 6: {
									int16_t accelZ = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipZ) {
										accelZ = I16T(-accelZ);
									}
									caerIMU6EventSetAccelZ(&state->currentIMU6Event, accelZ / state->imuAccelScale);
									break;
								}

									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 340) + 36.53
								case 8: {
									int16_t temp = I16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetTemp(&state->currentIMU6Event, (temp / 340.0f) + 36.53f);
									break;
								}

								case 10: {
									int16_t gyroX = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipX) {
										gyroX = I16T(-gyroX);
									}
									caerIMU6EventSetGyroX(&state->currentIMU6Event, gyroX / state->imuGyroScale);
									break;
								}

								case 12: {
									int16_t gyroY = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipY) {
										gyroY = I16T(-gyroY);
									}
									caerIMU6EventSetGyroY(&state->currentIMU6Event, gyroY / state->imuGyroScale);
									break;
								}

								case 14: {
									int16_t gyroZ = I16T((state->imuTmpData << 8) | misc8Data);
									if (state->imuFlipZ) {
										gyroZ = I16T(-gyroZ);
									}
									caerIMU6EventSetGyroZ(&state->currentIMU6Event, gyroZ / state->imuGyroScale);
									break;
								}
							}

							state->imuCount++;

							break;

						case 1:
							// APS ROI Size Part 1 (bits 15-8).
							// Here we just store the temporary value, and use it again
							// in the next case statement.
							state->apsROITmpData = U16T(misc8Data << 8);

							break;

						case 2: {
							// APS ROI Size Part 2 (bits 7-0).
							// Here we just store the values and re-use the four fields
							// sizeX/Y and positionX/Y to store endCol/Row and startCol/Row.
							// We then recalculate all the right values and set everything
							// up in START_FRAME.
							size_t apsROIRegion = state->apsROIUpdate >> 2;

							switch (state->apsROIUpdate & 0x03) {
								case 0:
									// START COLUMN
									state->apsROIPositionX[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
									break;

								case 1:
									// START ROW
									state->apsROIPositionY[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
									break;

								case 2:
									// END COLUMN
									state->apsROISizeX[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
									break;

								case 3:
									// END ROW
									state->apsROISizeY[apsROIRegion] = U16T(state->apsROITmpData | misc8Data);
									break;

								default:
									break;
							}

							// Jump to next type of APS info (col->row, start->end).
							state->apsROIUpdate++;

							break;
						}

						case 4: {
							// Microphone FIRST RIGHT.
							state->micRight = true;
							state->micCount = 1;
							state->micTmpData = misc8Data;
							break;
						}

						case 5: {
							// Microphone FIRST LEFT.
							state->micRight = false;
							state->micCount = 1;
							state->micTmpData = misc8Data;
							break;
						}

						case 6: {
							// Microphone SECOND.
							if (state->micCount != 1) {
								// Ignore incomplete samples.
								break;
							}

							state->micCount = 2;
							state->micTmpData = U16T(U32T(state->micTmpData << 8) | misc8Data);
							break;
						}

						case 7: {
							// Microphone THIRD.
							if (state->micCount != 2) {
								// Ignore incomplete samples.
								break;
							}

							state->micCount = 0;
							uint32_t micData = U32T(U32T(state->micTmpData << 8) | misc8Data);

							caerSampleEvent micSample = caerSampleEventPacketGetEvent(state->currentSamplePacket,
								state->currentSamplePacketPosition);
							caerSampleEventSetType(micSample, state->micRight);
							caerSampleEventSetSample(micSample, micData);
							caerSampleEventSetTimestamp(micSample, state->currentTimestamp);
							caerSampleEventValidate(micSample, state->currentSamplePacket);
							state->currentSamplePacketPosition++;
							break;
						}

						default:
							davisCommonLog(CAER_LOG_ERROR, handle, "Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 6: {
					// Misc 10bit data.
					uint8_t misc10Code = U8T((data & 0x0C00) >> 10);
					uint16_t misc10Data = U16T(data & 0x03FF);

					switch (misc10Code) {
						case 0:
							state->apsExposureFrameValue |= (U32T(misc10Data) << (10 * state->apsExposureFrameUpdate++));
							break;

						default:
							davisCommonLog(CAER_LOG_ERROR, handle, "Caught Misc10 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: { // Timestamp wrap
					// Detect big timestamp wrap-around.
					int64_t wrapJump = (TS_WRAP_ADD * data);
					int64_t wrapSum = I64T(state->wrapAdd) + wrapJump;

					if (wrapSum > I64T(INT32_MAX)) {
						// Reset wrapAdd at this point, so we can again
						// start detecting overruns of the 32bit value.
						// We reset not to zero, but to the remaining value after
						// multiple wrap-jumps are taken into account.
						int64_t wrapRemainder = wrapSum - I64T(INT32_MAX) - 1LL;
						state->wrapAdd = I32T(wrapRemainder);

						state->lastTimestamp = 0;
						state->currentTimestamp = state->wrapAdd;

						// Increment TSOverflow counter.
						state->wrapOverflow++;

						caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
							state->currentSpecialPacket, state->currentSpecialPacketPosition);
						caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
						caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
						caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
						state->currentSpecialPacketPosition++;

						// Commit packets to separate before wrap from after cleanly.
						tsBigWrap = true;
					}
					else {
						// Each wrap is 2^15 µs (~32ms), and we have
						// to multiply it with the wrap counter,
						// which is located in the data part of this
						// event.
						state->wrapAdd = I32T(wrapSum);

						state->lastTimestamp = state->currentTimestamp;
						state->currentTimestamp = state->wrapAdd;
						initContainerCommitTimestamp(state);

						// Check monotonicity of timestamps.
						checkStrictMonotonicTimestamp(handle);

						davisCommonLog(CAER_LOG_DEBUG, handle,
							"Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					}

					break;
				}

				default:
					davisCommonLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// forceCommit is already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = I32T(
			atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentPolarityPacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentFramePacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentIMU6PacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentSamplePacketPosition >= currentPacketContainerCommitSize));

		bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
			> state->currentPacketContainerCommitTimestamp;

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPolarityPacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT,
					(caerEventPacketHeader) state->currentPolarityPacket);

				state->currentPolarityPacket = NULL;
				state->currentPolarityPacketPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentSpecialPacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT,
					(caerEventPacketHeader) state->currentSpecialPacket);

				state->currentSpecialPacket = NULL;
				state->currentSpecialPacketPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentFramePacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, FRAME_EVENT,
					(caerEventPacketHeader) state->currentFramePacket);

				state->currentFramePacket = NULL;
				state->currentFramePacketPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentIMU6PacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, IMU6_EVENT,
					(caerEventPacketHeader) state->currentIMU6Packet);

				state->currentIMU6Packet = NULL;
				state->currentIMU6PacketPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentSamplePacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DAVIS_SAMPLE_POSITION,
					(caerEventPacketHeader) state->currentSamplePacket);

				state->currentSamplePacket = NULL;
				state->currentSamplePacketPosition = 0;
				emptyContainerCommit = false;
			}

			if (tsReset || tsBigWrap) {
				// Ignore all APS and IMU6 (composite) events, until a new APS or IMU6
				// Start event comes in, for the next packet.
				// This is to correctly support the forced packet commits that a TS reset,
				// or a TS big wrap, impose. Continuing to parse events would result
				// in a corrupted state of the first event in the new packet, as it would
				// be incomplete, incorrect and miss vital initialization data.
				// See APS and IMU6 END states for more details on a related issue.
				state->apsIgnoreEvents = true;
				state->imuIgnoreEvents = true;
			}

			// If the commit was triggered by a packet container limit being reached, we always
			// update the time related limit. The size related one is updated implicitly by size
			// being reset to zero after commit (new packets are empty).
			if (containerTimeCommit) {
				while (generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
					> state->currentPacketContainerCommitTimestamp) {
					state->currentPacketContainerCommitTimestamp += I32T(
						atomic_load_explicit( &state->maxPacketContainerInterval, memory_order_relaxed));
				}
			}

			// Filter out completely empty commits. This can happen when data is turned off,
			// but the timestamps are still going forward.
			if (emptyContainerCommit) {
				caerEventPacketContainerFree(state->currentPacketContainer);
				state->currentPacketContainer = NULL;
			}
			else {
				if (!ringBufferPut(state->dataExchangeBuffer, state->currentPacketContainer)) {
					// Failed to forward packet container, just drop it, it doesn't contain
					// any critical information anyway.
					davisCommonLog(CAER_LOG_INFO, handle, "Dropped EventPacket Container because ring-buffer full!");

					caerEventPacketContainerFree(state->currentPacketContainer);
					state->currentPacketContainer = NULL;
				}
				else {
					if (state->dataNotifyIncrease != NULL) {
						state->dataNotifyIncrease(state->dataNotifyUserPtr);
					}

					state->currentPacketContainer = NULL;
				}
			}

			// The only critical timestamp information to forward is the timestamp reset event.
			// The timestamp big-wrap can also (and should!) be detected by observing a packet's
			// tsOverflow value, not the special packet TIMESTAMP_WRAP event, which is only informative.
			// For the timestamp reset event (TIMESTAMP_RESET), we thus ensure that it is always
			// committed, and we send it alone, in its own packet container, to ensure it will always
			// be ordered after any other event packets in any processing or output stream.
			if (tsReset) {
				// Allocate packet container just for this event.
				caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(DAVIS_EVENT_TYPES);
				if (tsResetContainer == NULL) {
					davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset event packet container.");
					return;
				}

				// Allocate special packet just for this event.
				caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
					state->wrapOverflow);
				if (tsResetPacket == NULL) {
					davisCommonLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset special event packet.");
					return;
				}

				// Create timestamp reset event.
				caerSpecialEvent tsResetEvent = caerSpecialEventPacketGetEvent(tsResetPacket, 0);
				caerSpecialEventSetTimestamp(tsResetEvent, INT32_MAX);
				caerSpecialEventSetType(tsResetEvent, TIMESTAMP_RESET);
				caerSpecialEventValidate(tsResetEvent, tsResetPacket);

				// Assign special packet to packet container.
				caerEventPacketContainerSetEventPacket(tsResetContainer, SPECIAL_EVENT,
					(caerEventPacketHeader) tsResetPacket);

				// Reset MUST be committed, always, else downstream data processing and
				// outputs get confused if they have no notification of timestamps
				// jumping back go zero.
				while (!ringBufferPut(state->dataExchangeBuffer, tsResetContainer)) {
					// Prevent dead-lock if shutdown is requested and nothing is consuming
					// data anymore, but the ring-buffer is full (and would thus never empty),
					// thus blocking the USB handling thread in this loop.
					if (!usbDataTransfersAreRunning(&state->usbState)) {
						return;
					}
				}

				// Signal new container as usual.
				if (state->dataNotifyIncrease != NULL) {
					state->dataNotifyIncrease(state->dataNotifyUserPtr);
				}
			}
		}
	}
}

static void davisTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param) {
	// If any USB error happened, discard.
	if (status != LIBUSB_TRANSFER_COMPLETED) {
		return;
	}

	// Get new Master/Slave information from device. Done here to prevent deadlock
	// inside asynchronous callback.
	struct caer_davis_info *info = userDataPtr;

	atomic_thread_fence(memory_order_seq_cst);
	info->deviceIsMaster = param;
	atomic_thread_fence(memory_order_seq_cst);
}

uint16_t caerBiasVDACGenerate(const struct caer_bias_vdac vdacBias) {
	// Build up bias value from all its components.
	uint16_t biasValue = U16T((vdacBias.voltageValue & 0x3F) << 0);
	biasValue = U16T(biasValue | ((vdacBias.currentValue & 0x07) << 6));

	return (biasValue);
}

struct caer_bias_vdac caerBiasVDACParse(const uint16_t vdacBias) {
	struct caer_bias_vdac biasValue;

	// Decompose bias integer into its parts.
	biasValue.voltageValue = vdacBias & 0x3F;
	biasValue.currentValue = (vdacBias >> 6) & 0x07;

	return (biasValue);
}

uint16_t caerBiasCoarseFineGenerate(const struct caer_bias_coarsefine coarseFineBias) {
	uint16_t biasValue = 0;

	// Build up bias value from all its components.
	if (coarseFineBias.enabled) {
		biasValue |= 0x01;
	}
	if (coarseFineBias.sexN) {
		biasValue |= 0x02;
	}
	if (coarseFineBias.typeNormal) {
		biasValue |= 0x04;
	}
	if (coarseFineBias.currentLevelNormal) {
		biasValue |= 0x08;
	}

	biasValue = U16T(biasValue | ((coarseFineBias.fineValue & 0xFF) << 4));
	biasValue = U16T(biasValue | ((coarseFineBias.coarseValue & 0x07) << 12));

	return (biasValue);
}

struct caer_bias_coarsefine caerBiasCoarseFineParse(const uint16_t coarseFineBias) {
	struct caer_bias_coarsefine biasValue;

	// Decompose bias integer into its parts.
	biasValue.enabled = (coarseFineBias & 0x01);
	biasValue.sexN = (coarseFineBias & 0x02);
	biasValue.typeNormal = (coarseFineBias & 0x04);
	biasValue.currentLevelNormal = (coarseFineBias & 0x08);
	biasValue.fineValue = U8T(coarseFineBias >> 4) & 0xFF;
	biasValue.coarseValue = U8T(coarseFineBias >> 12) & 0x07;

	return (biasValue);
}

uint16_t caerBiasShiftedSourceGenerate(const struct caer_bias_shiftedsource shiftedSourceBias) {
	uint16_t biasValue = 0;

	if (shiftedSourceBias.operatingMode == HI_Z) {
		biasValue |= 0x01;
	}
	else if (shiftedSourceBias.operatingMode == TIED_TO_RAIL) {
		biasValue |= 0x02;
	}

	if (shiftedSourceBias.voltageLevel == SINGLE_DIODE) {
		biasValue |= (0x01 << 2);
	}
	else if (shiftedSourceBias.voltageLevel == DOUBLE_DIODE) {
		biasValue |= (0x02 << 2);
	}

	biasValue = U16T(biasValue | ((shiftedSourceBias.refValue & 0x3F) << 4));
	biasValue = U16T(biasValue | ((shiftedSourceBias.regValue & 0x3F) << 10));

	return (biasValue);
}

struct caer_bias_shiftedsource caerBiasShiftedSourceParse(const uint16_t shiftedSourceBias) {
	struct caer_bias_shiftedsource biasValue;

	// Decompose bias integer into its parts.
	biasValue.operatingMode =
		(shiftedSourceBias & 0x01) ? (HI_Z) : ((shiftedSourceBias & 0x02) ? (TIED_TO_RAIL) : (SHIFTED_SOURCE));
	biasValue.voltageLevel =
		((shiftedSourceBias >> 2) & 0x01) ?
			(SINGLE_DIODE) : (((shiftedSourceBias >> 2) & 0x02) ? (DOUBLE_DIODE) : (SPLIT_GATE));
	biasValue.refValue = (shiftedSourceBias >> 4) & 0x3F;
	biasValue.regValue = (shiftedSourceBias >> 10) & 0x3F;

	return (biasValue);
}
