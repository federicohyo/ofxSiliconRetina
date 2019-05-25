#ifndef LIBCAER_SRC_DAVIS_COMMON_H_
#define LIBCAER_SRC_DAVIS_COMMON_H_

#include "devices/davis.h"
#include "devices/device_discover.h"

#include "filters/dvs_noise.h"

#include "autoexposure.h"
#include "container_generation.h"
#include "data_exchange.h"
#include "frame_utils.h"
#include "spi_config_interface.h"

#include <math.h>
#include <stdatomic.h>

/**
 * Enable APS frame debugging by only looking at the reset or signal
 * frames, and not at the resulting correlated frame.
 * Supported values:
 * 0 - normal output (ROI region 0), no debug (default)
 * 1 - normal output (ROI region 0), and in addition both reset and
 *     signal separately (marked as ROI regions 1 for reset and 2
 *     for signal respectively)
 */
#define APS_DEBUG_FRAME 0

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET 0
#define APS_READOUT_SIGNAL 1

#define APS_ADC_DEPTH 10

#define DVS_HOTPIXEL_HW_MAX 8

#define IMU_TYPE_TEMP 0x01
#define IMU_TYPE_GYRO 0x02
#define IMU_TYPE_ACCEL 0x04
#define IMU_TOTAL_COUNT 14

#define DAVIS_EVENT_TYPES 4

#define DAVIS_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_FRAME_DEFAULT_SIZE 8
#define DAVIS_IMU_DEFAULT_SIZE 64

struct davis_common_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastY;
		uint16_t sizeX;
		uint16_t sizeY;
		bool invertXY;
		struct {
			atomic_bool autoTrainRunning;
			caerFilterDVSNoise noiseFilter;
		} pixelFilterAutoTrain;
	} dvs;
	struct {
		// APS specific fields
		uint16_t sizeX;
		uint16_t sizeY;
		bool invertXY;
		bool flipX;
		bool flipY;
		bool ignoreEvents;
		bool globalShutter;
		uint16_t currentReadoutType;
		uint16_t countX[APS_READOUT_TYPES_NUM];
		uint16_t countY[APS_READOUT_TYPES_NUM];
		uint16_t expectedCountX;
		uint16_t expectedCountY;
		struct {
			caerFrameEvent currentEvent;
			atomic_uint_fast8_t mode;
#if APS_DEBUG_FRAME == 1
			uint16_t *resetPixels;
			uint16_t *signalPixels;
#endif
		} frame;
		struct {
			// Temporary values from device.
			uint16_t tmpData;
			uint16_t update;
			// Parameters for frame parsing.
			uint16_t positionX;
			uint16_t positionY;
			uint16_t sizeX;
			uint16_t sizeY;
		} roi;
		struct {
			bool offsetDirection; // 0 is increasing, 1 is decreasing.
			int16_t offset;
		} cDavisSupport;
		struct {
			uint8_t tmpData;
			uint32_t currentFrameExposure;
			uint32_t lastSetExposure;
			atomic_bool enabled;
			struct auto_exposure_state state;
		} autoExposure;
	} aps;
	struct {
		// IMU specific fields
		bool ignoreEvents;
		bool flipX;
		bool flipY;
		bool flipZ;
		uint8_t type;
		uint8_t count;
		uint8_t tmpData;
		float accelScale;
		float gyroScale;
		// Current composite events, for later copy, to not loose them on commits.
		struct caer_imu6_event currentEvent;
	} imu;
	// Packet Container state
	struct container_generation container;
	struct {
		// Polarity Packet state
		caerPolarityEventPacket polarity;
		int32_t polarityPosition;
		// Frame Packet state
		caerFrameEventPacket frame;
		int32_t framePosition;
		// IMU6 Packet state
		caerIMU6EventPacket imu6;
		int32_t imu6Position;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
	// Device timing data.
	struct {
		uint16_t logicClock;
		uint16_t adcClock;
		uint16_t usbClock;
		uint16_t clockDeviationFactor;
		float logicClockActual;
		float adcClockActual;
		float usbClockActual;
	} deviceClocks;
};

typedef struct davis_common_state *davisCommonState;

struct davis_common_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_davis_info info;
	// State for data management
	struct davis_common_state state;
	// Pointer to SPI configuration state, depends on implementor.
	void *spiConfigPtr;
};

typedef struct davis_common_handle *davisCommonHandle;

static void davisLog(enum caer_log_level logLevel, davisCommonHandle handle, const char *format, ...)
	ATTRIBUTE_FORMAT(3);
static void davisCommonInit(davisCommonHandle handle);
static bool davisCommonSendDefaultFPGAConfig(davisCommonHandle handle);
static bool davisCommonSendDefaultChipConfig(davisCommonHandle handle);
static bool davisCommonConfigSet(davisCommonHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param);
static bool davisCommonConfigGet(davisCommonHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param);
static bool davisCommonDataStart(davisCommonHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr);
static void davisCommonDataStop(davisCommonHandle handle);
static void davisCommonEventTranslator(
	davisCommonHandle handle, const uint8_t *buffer, size_t bufferSize, atomic_uint_fast32_t *transfersRunning);
static void davisCommonTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param);

static void davisLog(enum caer_log_level logLevel, davisCommonHandle handle, const char *format, ...) {
	// Only log messages above the specified severity level.
	uint8_t systemLogLevel = atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed);

	if (logLevel > systemLogLevel) {
		return;
	}

	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(systemLogLevel, logLevel, handle->info.deviceString, format, argumentList);
	va_end(argumentList);
}

static inline void freeAllDataMemory(davisCommonState state) {
	dataExchangeDestroy(&state->dataExchange);

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPackets.polarity != NULL) {
		free(&state->currentPackets.polarity->packetHeader);
		state->currentPackets.polarity = NULL;

		containerGenerationSetPacket(&state->container, POLARITY_EVENT, NULL);
	}

	if (state->currentPackets.special != NULL) {
		free(&state->currentPackets.special->packetHeader);
		state->currentPackets.special = NULL;

		containerGenerationSetPacket(&state->container, SPECIAL_EVENT, NULL);
	}

	if (state->currentPackets.frame != NULL) {
		free(&state->currentPackets.frame->packetHeader);
		state->currentPackets.frame = NULL;

		containerGenerationSetPacket(&state->container, FRAME_EVENT, NULL);
	}

	if (state->currentPackets.imu6 != NULL) {
		free(&state->currentPackets.imu6->packetHeader);
		state->currentPackets.imu6 = NULL;

		containerGenerationSetPacket(&state->container, IMU6_EVENT, NULL);
	}

	containerGenerationDestroy(&state->container);

	if (state->aps.frame.currentEvent != NULL) {
		free(state->aps.frame.currentEvent);
		state->aps.frame.currentEvent = NULL;
	}

#if APS_DEBUG_FRAME == 1
	if (state->aps.frame.resetPixels != NULL) {
		free(state->aps.frame.resetPixels);
		state->aps.frame.resetPixels = NULL;
	}

	if (state->aps.frame.signalPixels != NULL) {
		free(state->aps.frame.signalPixels);
		state->aps.frame.signalPixels = NULL;
	}
#endif
}

static inline bool ensureSpaceForEvents(
	caerEventPacketHeader *packet, size_t position, size_t numEvents, davisCommonHandle handle) {
	if ((position + numEvents) <= (size_t) caerEventPacketHeaderGetEventCapacity(*packet)) {
		return (true);
	}

	caerEventPacketHeader grownPacket
		= caerEventPacketGrow(*packet, caerEventPacketHeaderGetEventCapacity(*packet) * 2);
	if (grownPacket == NULL) {
		davisLog(CAER_LOG_CRITICAL, handle, "Failed to grow event packet of type %d.",
			caerEventPacketHeaderGetEventType(*packet));
		return (false);
	}

	*packet = grownPacket;
	return (true);
}

static inline void apsInitFrame(davisCommonHandle handle) {
	davisCommonState state = &handle->state;

	state->aps.ignoreEvents                      = false;
	state->aps.autoExposure.tmpData              = 0;
	state->aps.autoExposure.currentFrameExposure = 0;
	state->aps.roi.tmpData                       = 0;
	state->aps.roi.update                        = 0;

	state->aps.currentReadoutType = APS_READOUT_RESET;
	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		state->aps.countX[i] = 0;
		state->aps.countY[i] = 0;
	}

	// Write out start of frame timestamp.
	caerFrameEventSetTSStartOfFrame(state->aps.frame.currentEvent, state->timestamps.current);

	// Send APS info event out (as special event).
	if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
			(size_t) state->currentPackets.specialPosition, 1, handle)) {
		caerSpecialEvent currentSpecialEvent
			= caerSpecialEventPacketGetEvent(state->currentPackets.special, state->currentPackets.specialPosition);
		caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
		caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_START);
		caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
		state->currentPackets.specialPosition++;
	}
}

static inline void apsROIUpdateSizes(davisCommonHandle handle) {
	davisCommonState state = &handle->state;

	// Calculate APS ROI sizes.
	uint16_t startColumn = state->aps.roi.positionX;
	uint16_t startRow    = state->aps.roi.positionY;
	uint16_t endColumn   = state->aps.roi.sizeX;
	uint16_t endRow      = state->aps.roi.sizeY;

	// Position is already set to startCol/Row, so we don't have to reset
	// it here. We only have to calculate size from start and end Col/Row.
	state->aps.roi.sizeX = U16T(endColumn + 1 - startColumn);
	state->aps.roi.sizeY = U16T(endRow + 1 - startRow);

	// Sanity check.
	if ((state->aps.roi.sizeX == 0) || (state->aps.roi.sizeX > handle->info.apsSizeX)) {
		davisLog(CAER_LOG_ALERT, handle, "ROI X size incorrect - SizeX: %d.", state->aps.roi.sizeX);

		state->aps.roi.sizeX = U16T(handle->info.apsSizeX);
	}

	if ((state->aps.roi.sizeY == 0) || (state->aps.roi.sizeY > handle->info.apsSizeY)) {
		davisLog(CAER_LOG_ALERT, handle, "ROI Y size incorrect - SizeY: %d.", state->aps.roi.sizeY);

		state->aps.roi.sizeY = U16T(handle->info.apsSizeY);
	}

	if ((state->aps.roi.positionX + state->aps.roi.sizeX) > handle->info.apsSizeX) {
		davisLog(CAER_LOG_ALERT, handle, "ROI X position incorrect - PosX: %d.", state->aps.roi.positionX);

		state->aps.roi.positionX = 0;
	}

	if ((state->aps.roi.positionY + state->aps.roi.sizeY) > handle->info.apsSizeY) {
		davisLog(CAER_LOG_ALERT, handle, "ROI Y position incorrect - PosY: %d.", state->aps.roi.positionY);

		state->aps.roi.positionY = 0;
	}

	if (state->aps.invertXY) {
		state->aps.expectedCountX = state->aps.roi.sizeY;
		state->aps.expectedCountY = state->aps.roi.sizeX;
	}
	else {
		state->aps.expectedCountX = state->aps.roi.sizeX;
		state->aps.expectedCountY = state->aps.roi.sizeY;
	}
}

static inline void apsUpdateFrame(davisCommonHandle handle, uint16_t data) {
	davisCommonState state = &handle->state;

	uint16_t xPos = (state->aps.flipX)
						? (U16T(state->aps.expectedCountX - 1 - state->aps.countX[state->aps.currentReadoutType]))
						: (state->aps.countX[state->aps.currentReadoutType]);
	uint16_t yPos = (state->aps.flipY)
						? (U16T(state->aps.expectedCountY - 1 - state->aps.countY[state->aps.currentReadoutType]))
						: (state->aps.countY[state->aps.currentReadoutType]);

	if (IS_DAVIS640H(handle->info.chipID)) {
		yPos = U16T(yPos + state->aps.cDavisSupport.offset);
	}

	if (state->aps.invertXY) {
		SWAP_VAR(uint16_t, xPos, yPos);
	}

	size_t pixelPosition = (size_t)(yPos * state->aps.roi.sizeX) + xPos;

	// Standard CDS support.
	bool isCDavisGS = (IS_DAVIS640H(handle->info.chipID) && state->aps.globalShutter);

	if (((state->aps.currentReadoutType == APS_READOUT_RESET) && (!isCDavisGS))
		|| ((state->aps.currentReadoutType == APS_READOUT_SIGNAL) && isCDavisGS)) {
		state->aps.frame.currentEvent->pixels[pixelPosition] = data;
	}
	else {
		uint16_t resetValue  = 0;
		uint16_t signalValue = 0;

		if (isCDavisGS) {
			// DAVIS640H GS has inverted samples, signal read comes first
			// and was stored above inside state->aps.currentResetFrame.
			resetValue  = data;
			signalValue = state->aps.frame.currentEvent->pixels[pixelPosition];
		}
		else {
			resetValue  = state->aps.frame.currentEvent->pixels[pixelPosition];
			signalValue = data;
		}

		int32_t pixelValue = 0;

		if ((resetValue < 384) || (signalValue == 0)) {
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

		// Normalize the ADC value to 16bit generic depth. This depends on ADC used.
		pixelValue = pixelValue << (16 - APS_ADC_DEPTH);

		state->aps.frame.currentEvent->pixels[pixelPosition] = htole16(U16T(pixelValue));
	}

	// DAVIS640H support: first 320 pixels are even, then odd.
	if (IS_DAVIS640H(handle->info.chipID)) {
		if (state->aps.cDavisSupport.offsetDirection == 0) { // Increasing
			state->aps.cDavisSupport.offset++;

			if (state->aps.cDavisSupport.offset == 321) {
				// Switch to decreasing after last even pixel.
				state->aps.cDavisSupport.offsetDirection = 1;
				state->aps.cDavisSupport.offset          = 318;
			}
		}
		else { // Decreasing
			state->aps.cDavisSupport.offset = I16T(state->aps.cDavisSupport.offset - 3);
		}
	}

// Separate debug support.
#if APS_DEBUG_FRAME == 1
	// Check for overflow.
	data = (data > 1023) ? (1023) : (data);

	// Normalize the ADC value to 16bit generic depth. This depends on ADC used.
	data = U16T(data << (16 - APS_ADC_DEPTH));

	// Reset read, put into resetPixels here.
	if (state->aps.currentReadoutType == APS_READOUT_RESET) {
		state->aps.frame.resetPixels[pixelPosition] = htole16(data);
	}

	// Signal read, put into pixels here.
	if (state->aps.currentReadoutType == APS_READOUT_SIGNAL) {
		state->aps.frame.signalPixels[pixelPosition] = htole16(data);
	}

	davisLog(CAER_LOG_DEBUG, handle,
		"APS ADC Sample: column=%" PRIu16 ", row=%" PRIu16 ", index=%zu, data=%" PRIu16 ".",
		state->aps.countX[state->aps.currentReadoutType], state->aps.countY[state->aps.currentReadoutType],
		pixelPosition, data);
#endif
}

static inline bool apsEndFrame(davisCommonHandle handle) {
	davisCommonState state = &handle->state;

	bool validFrame = true;

	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		davisLog(CAER_LOG_DEBUG, handle, "APS Frame End: CountX[%zu] is %d.", i, state->aps.countX[i]);

		if (state->aps.countX[i] != state->aps.expectedCountX) {
			davisLog(CAER_LOG_ERROR, handle, "APS Frame End - %zu: wrong column count %d detected, expected %d.", i,
				state->aps.countX[i], state->aps.expectedCountX);
			validFrame = false;
		}
	}

	// Write out end of frame timestamp.
	caerFrameEventSetTSEndOfFrame(state->aps.frame.currentEvent, state->timestamps.current);

	// Send APS info event out (as special event).
	if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
			(size_t) state->currentPackets.specialPosition, 1, handle)) {
		caerSpecialEvent currentSpecialEvent
			= caerSpecialEventPacketGetEvent(state->currentPackets.special, state->currentPackets.specialPosition);
		caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
		caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_END);
		caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
		state->currentPackets.specialPosition++;
	}

	return (validFrame);
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +-2 g - 16384 LSB/g
	// 1 - +-4 g - 8192 LSB/g
	// 2 - +-8 g - 4096 LSB/g
	// 3 - +-16 g - 2048 LSB/g
	float accelScale = 65536.0F / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Gyroscope scale is:
	// 0 - +-250 °/s - 131 LSB/°/s
	// 1 - +-500 °/s - 65.5 LSB/°/s
	// 2 - +-1000 °/s - 32.8 LSB/°/s
	// 3 - +-2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0F / (float) U32T(500 * (1 << imuGyroScale));

	return (gyroScale);
}

static void davisCommonInit(davisCommonHandle handle) {
	davisCommonState state = &handle->state;

	// Populate info variables based on data from device.
	uint32_t param32 = 0;

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
	handle->info.chipID = I16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	state->deviceClocks.logicClock = U16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_ADC_CLOCK, &param32);
	state->deviceClocks.adcClock = U16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_USB_CLOCK, &param32);
	state->deviceClocks.usbClock = U16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_CLOCK_DEVIATION, &param32);
	state->deviceClocks.clockDeviationFactor = U16T(param32);

	// Calculate actual clock frequencies.
	state->deviceClocks.logicClockActual = (float) ((double) state->deviceClocks.logicClock
													* ((double) state->deviceClocks.clockDeviationFactor / 1000.0));
	state->deviceClocks.adcClockActual   = (float) ((double) state->deviceClocks.adcClock
                                                  * ((double) state->deviceClocks.clockDeviationFactor / 1000.0));
	state->deviceClocks.usbClockActual   = (float) ((double) state->deviceClocks.usbClock
                                                  * ((double) state->deviceClocks.clockDeviationFactor / 1000.0));

	davisLog(CAER_LOG_DEBUG, handle, "Clock frequencies: LOGIC %f, ADC %f, USB %f.",
		(double) state->deviceClocks.logicClockActual, (double) state->deviceClocks.adcClockActual,
		(double) state->deviceClocks.usbClockActual);

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_PIXEL_FILTER, &param32);
	handle->info.dvsHasPixelFilter = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_BACKGROUND_ACTIVITY_FILTER, &param32);
	handle->info.dvsHasBackgroundActivityFilter = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_ROI_FILTER, &param32);
	handle->info.dvsHasROIFilter = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_SKIP_FILTER, &param32);
	handle->info.dvsHasSkipFilter = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_POLARITY_FILTER, &param32);
	handle->info.dvsHasPolarityFilter = param32;
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_STATISTICS, &param32);
	handle->info.dvsHasStatistics = param32;

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLOR_FILTER, &param32);
	handle->info.apsColorFilter = U8T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_GLOBAL_SHUTTER, &param32);
	handle->info.apsHasGlobalShutter = param32;

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_HAS_GENERATOR, &param32);
	handle->info.extInputHasGenerator = param32;

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_HAS_STATISTICS, &param32);
	handle->info.muxHasStatistics = param32;

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_COLUMNS, &param32);
	state->dvs.sizeX = U16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_ROWS, &param32);
	state->dvs.sizeY = U16T(param32);

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ORIENTATION_INFO, &param32);
	state->dvs.invertXY = param32 & 0x04;

	davisLog(CAER_LOG_DEBUG, handle, "DVS Size X: %d, Size Y: %d, Invert: %d.", state->dvs.sizeX, state->dvs.sizeY,
		state->dvs.invertXY);

	if (state->dvs.invertXY) {
		handle->info.dvsSizeX = I16T(state->dvs.sizeY);
		handle->info.dvsSizeY = I16T(state->dvs.sizeX);
	}
	else {
		handle->info.dvsSizeX = I16T(state->dvs.sizeX);
		handle->info.dvsSizeY = I16T(state->dvs.sizeY);
	}

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_COLUMNS, &param32);
	state->aps.sizeX = U16T(param32);
	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_ROWS, &param32);
	state->aps.sizeY = U16T(param32);

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ORIENTATION_INFO, &param32);
	state->aps.invertXY = param32 & 0x04;
	state->aps.flipX    = param32 & 0x02;
	state->aps.flipY    = param32 & 0x01;

	davisLog(CAER_LOG_DEBUG, handle, "APS Size X: %d, Size Y: %d, Invert: %d, Flip X: %d, Flip Y: %d.",
		state->aps.sizeX, state->aps.sizeY, state->aps.invertXY, state->aps.flipX, state->aps.flipY);

	if (state->aps.invertXY) {
		handle->info.apsSizeX = I16T(state->aps.sizeY);
		handle->info.apsSizeY = I16T(state->aps.sizeX);
	}
	else {
		handle->info.apsSizeX = I16T(state->aps.sizeX);
		handle->info.apsSizeY = I16T(state->aps.sizeY);
	}

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_TYPE, &param32);
	handle->info.imuType = U8T(param32);

	spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ORIENTATION_INFO, &param32);
	state->imu.flipX = param32 & 0x04;
	state->imu.flipY = param32 & 0x02;
	state->imu.flipZ = param32 & 0x01;

	davisLog(CAER_LOG_DEBUG, handle, "IMU Flip X: %d, Flip Y: %d, Flip Z: %d.", state->imu.flipX, state->imu.flipY,
		state->imu.flipZ);
}

static bool davisCommonSendDefaultFPGAConfig(davisCommonHandle handle) {
	davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
	davisCommonConfigSet(handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL, true);

	davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL, false);
	if (handle->info.dvsHasPixelFilter) {
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, U32T(handle->info.dvsSizeX));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, U32T(handle->info.dvsSizeY));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, U32T(handle->info.dvsSizeX));
	}
	if (handle->info.dvsHasBackgroundActivityFilter) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, true);
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, 8); // in 250µs blocks (so 2ms)
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, false);
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, 1); // in 250µs blocks (so 250µs)
	}
	if (handle->info.dvsHasROIFilter) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, 0);
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, 0);
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, U32T(handle->info.dvsSizeX - 1));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, U32T(handle->info.dvsSizeY - 1));
	}
	if (handle->info.dvsHasSkipFilter) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, 5);
	}
	if (handle->info.dvsHasPolarityFilter) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, false);
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, false); // Suppress OFF events.
	}

	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL, true);
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, handle->info.apsHasGlobalShutter);
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, U32T(handle->info.apsSizeX - 1));
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, U32T(handle->info.apsSizeY - 1));
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, APS_FRAME_DEFAULT);
	davisCommonConfigSet(
		handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4000); // in µs, converted to cycles @ ADCClock later
	davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL,
		40000); // in µs, converted to cycles @ ADCClock later

	if (IS_DAVIS640H(handle->info.chipID)) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_TRANSFER, 1500);   // in cycles @ ADCClock
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_RSFDSETTLE, 900);  // in cycles @ ADCClock
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSPDRESET, 900);   // in cycles @ ADCClock
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSRESETFALL, 900); // in cycles @ ADCClock
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSTXFALL, 900);    // in cycles @ ADCClock
		davisCommonConfigSet(handle, DAVIS_CONFIG_APS, DAVIS640H_CONFIG_APS_GSFDRESET, 900);   // in cycles @ ADCClock
	}

	davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);     // Sampling rate: 1KHz.
	davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_DLPF, 1);              // FS: 1KHz.
	davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, ACCEL_4G); // +- 4 g.
	davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_DLPF, 1);               // FS: 1KHz.
	davisCommonConfigSet(handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, GYRO_500DPS); // +- 500 °/s

	davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, true);
	davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, true);
	davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
		10); // in µs, converted to cycles @ LogicClock later

	if (handle->info.extInputHasGenerator) {
		// Disable generator by default. Has to be enabled manually after sendDefaultConfig() by user!
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
			10); // in µs, converted to cycles @ LogicClock later
		davisCommonConfigSet(handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
			5); // in µs, converted to cycles @ LogicClock later
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);
	}

	return (true);
}

#define CF_N_TYPE(COARSE, FINE)                                                                      \
	(struct caer_bias_coarsefine) {                                                                  \
		.coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, .typeNormal = true, \
		.currentLevelNormal = true                                                                   \
	}

#define CF_P_TYPE(COARSE, FINE)                                                                       \
	(struct caer_bias_coarsefine) {                                                                   \
		.coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, .typeNormal = true, \
		.currentLevelNormal = true                                                                    \
	}

#define CF_N_TYPE_CAS(COARSE, FINE)                                                                   \
	(struct caer_bias_coarsefine) {                                                                   \
		.coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, .typeNormal = false, \
		.currentLevelNormal = true                                                                    \
	}

/*
 * #define CF_P_TYPE_CAS(COARSE, FINE) (struct caer_bias_coarsefine) \
 *	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, \
 *	.typeNormal = false, .currentLevelNormal = true }
 */

#define CF_N_TYPE_OFF(COARSE, FINE)                                                                   \
	(struct caer_bias_coarsefine) {                                                                   \
		.coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = true, .typeNormal = true, \
		.currentLevelNormal = true                                                                    \
	}

#define CF_P_TYPE_OFF(COARSE, FINE)                                                                    \
	(struct caer_bias_coarsefine) {                                                                    \
		.coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = false, .typeNormal = true, \
		.currentLevelNormal = true                                                                     \
	}

#define SHIFTSOURCE(REF, REG, OPMODE)                                                         \
	(struct caer_bias_shiftedsource) {                                                        \
		.refValue = REF, .regValue = REG, .operatingMode = OPMODE, .voltageLevel = SPLIT_GATE \
	}

#define VDAC(VOLT, CURR)                           \
	(struct caer_bias_vdac) {                      \
		.voltageValue = VOLT, .currentValue = CURR \
	}

static bool davisCommonSendDefaultChipConfig(davisCommonHandle handle) {
	// Default bias configuration.
	if (IS_DAVIS240(handle->info.chipID)) {
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 39)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 0)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSCASEPC,
			caerBiasCoarseFineGenerate(CF_N_TYPE_CAS(5, 185)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFCASBNC,
			caerBiasCoarseFineGenerate(CF_N_TYPE_CAS(5, 115)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSROSFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LOCALBUFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PIXINVBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 58)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 16)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 25)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUXBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_AEPUYBP, caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFTHRBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_IFREFRBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PADFOLLBN, caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_APSOVERFLOWLEVELBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 253)));

		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_BIASBUFFER, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));

		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
	}

	if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID) || IS_DAVIS346(handle->info.chipID)
		|| IS_DAVIS640(handle->info.chipID)) {
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL, caerBiasVDACGenerate(VDAC(27, 6)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSCAS, caerBiasVDACGenerate(VDAC(21, 6)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFHIGH, caerBiasVDACGenerate(VDAC(32, 7)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFLOW, caerBiasVDACGenerate(VDAC(1, 7)));

		if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
			// Only DAVIS346 and 640 have ADC testing.
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE, caerBiasVDACGenerate(VDAC(21, 7)));
		}

		if (IS_DAVIS208(handle->info.chipID)) {
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_RESETHIGHPASS, caerBiasVDACGenerate(VDAC(63, 7)));
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSS, caerBiasVDACGenerate(VDAC(11, 5)));

			davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REGBIASBP,
				caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS208_CONFIG_BIAS_REFSSBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 20)));
		}

		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LOCALBUFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PADFOLLBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(7, 215)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 39)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 1)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PIXINVBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 58)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 16)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 25)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_READOUTBUFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSROSFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCCOMPBP, caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_COLSELLOWBN, caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DACBUFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUXBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUYBP, caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFREFRBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFTHRBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));

		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_BIASBUFFER, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));

		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));

		// This comes last here so it overrides previous settings for 640 only.
		if (IS_DAVIS640(handle->info.chipID)) {
			// Slow down pixels for big 640x480 array, to avoid overwhelming the AER bus.
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 3)));
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 1)));
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 155)));
			davisCommonConfigSet(
				handle, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(1, 4)));

			davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640_CONFIG_BIAS_BIASBUFFER,
				caerBiasCoarseFineGenerate(CF_N_TYPE(6, 125)));
		}
	}

	if (IS_DAVIS640H(handle->info.chipID)) {
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSCAS, caerBiasVDACGenerate(VDAC(21, 4)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG1LO, caerBiasVDACGenerate(VDAC(63, 4)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OVG2LO, caerBiasVDACGenerate(VDAC(0, 0)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_TX2OVG2HI, caerBiasVDACGenerate(VDAC(63, 0)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_GND07, caerBiasVDACGenerate(VDAC(13, 4)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCTESTVOLTAGE, caerBiasVDACGenerate(VDAC(21, 0)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFHIGH, caerBiasVDACGenerate(VDAC(46, 7)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCREFLOW, caerBiasVDACGenerate(VDAC(3, 7)));

		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFREFRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 255)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_IFTHRBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 255)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LOCALBUFBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(5, 164)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PADFOLLBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(7, 209)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PIXINVBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 164)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DIFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(3, 75)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 95)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(2, 41)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 88)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(1, 173)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_REFRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 62)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYBIASBUFFERBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(6, 128)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ARRAYLOGICBUFFERBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_FALLTIMEBN, caerBiasCoarseFineGenerate(CF_N_TYPE(7, 41)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_RISETIMEBP, caerBiasCoarseFineGenerate(CF_P_TYPE(6, 162)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_READOUTBUFBP,
			caerBiasCoarseFineGenerate(CF_P_TYPE_OFF(6, 20)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_APSROSFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(7, 82)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_ADCCOMPBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 159)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_DACBUFBP, caerBiasCoarseFineGenerate(CF_P_TYPE(6, 194)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_LCOLTIMEOUTBN,
			caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPDBN, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUXBP, caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_AEPUYBP, caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));

		davisCommonConfigSet(
			handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_BIASBUFFER, caerBiasCoarseFineGenerate(CF_N_TYPE(6, 251)));

		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSP,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, TIED_TO_RAIL)));
		davisCommonConfigSet(handle, DAVIS_CONFIG_BIAS, DAVIS640H_CONFIG_BIAS_SSN,
			caerBiasShiftedSourceGenerate(SHIFTSOURCE(2, 33, SHIFTED_SOURCE)));
	}

	// Default chip configuration.
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX0, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX1, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX2, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX3, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX0, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX1, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX2, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_BIASMUX0, 0);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETCALIBNEURON, true);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON, false);
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETTESTPIXEL, true);
	davisCommonConfigSet(
		handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_AERNAROW, false); // Use nArow in the AER state machine.
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_USEAOUT,
		false); // Enable analog pads for aMUX output (testing).

	// No GlobalShutter flag set here, we already set it above for the APS GS flag,
	// and that is automatically propagated to the chip config shift-register in
	// configSet() and kept in sync.

	// Special extra pixels control for DAVIS240 A/B.
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL, false);

	// Select which gray counter to use with the internal ADC: '0' means the external gray counter is used, which
	// has to be supplied off-chip. '1' means the on-chip gray counter is used instead.
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER, 1);

	// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
	// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
	davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS346_CONFIG_CHIP_TESTADC, false);

	if (IS_DAVIS208(handle->info.chipID)) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTBIASREFSS, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTSENSE, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTPOSFB, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS208_CONFIG_CHIP_SELECTHIGHPASS, false);
	}

	if (IS_DAVIS640H(handle->info.chipID)) {
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG1LO, true);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTOVG2LO, false);
		davisCommonConfigSet(handle, DAVIS_CONFIG_CHIP, DAVIS640H_CONFIG_CHIP_ADJUSTTX2OVG2HI, false);
	}

	return (true);
}

static bool davisCommonConfigSet(davisCommonHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	davisCommonState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigSet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigSet(&state->container, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					atomic_store(&state->deviceLogLevel, U8T(param));
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
				case DAVIS_CONFIG_MUX_RUN_CHIP:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						struct spi_config_params spiMultiConfig[2];

						spiMultiConfig[0].moduleAddr = DAVIS_CONFIG_MUX;
						spiMultiConfig[0].paramAddr  = DAVIS_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[0].param      = true;

						spiMultiConfig[1].moduleAddr = DAVIS_CONFIG_MUX;
						spiMultiConfig[1].paramAddr  = DAVIS_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[1].param      = false;

						return (spiConfigSendMultiple(handle->spiConfigPtr, spiMultiConfig, 2));
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
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW:
					if (handle->info.dvsHasROIFilter) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS:
				case DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY:
					if (handle->info.dvsHasSkipFilter) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN:
				case DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS:
				case DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE:
					if (handle->info.dvsHasPolarityFilter) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN:
					if (handle->info.dvsHasPixelFilter) {
						atomic_store(&handle->state.dvs.pixelFilterAutoTrain.autoTrainRunning, param);
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
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_START_COLUMN_0:
				case DAVIS_CONFIG_APS_START_ROW_0:
				case DAVIS_CONFIG_APS_END_COLUMN_0:
				case DAVIS_CONFIG_APS_END_ROW_0:
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Exposure and Frame Interval are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					if (!atomic_load(&state->aps.autoExposure.enabled)) {
						state->aps.autoExposure.lastSetExposure = param;

						float exposureCC = roundf((float) param * state->deviceClocks.adcClockActual);
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, U32T(exposureCC)));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_FRAME_INTERVAL: {
					// Exposure and Frame Interval are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					float intervalCC = roundf((float) param * state->deviceClocks.adcClockActual);
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, U32T(intervalCC)));
					break;
				}

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						// Keep in sync with chip config module GlobalShutter parameter.
						if (!IS_DAVIS640H(handle->info.chipID)) {
							if (!spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP,
									DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER, param)) {
								return (false);
							}
						}

						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS640H_CONFIG_APS_TRANSFER:
				case DAVIS640H_CONFIG_APS_RSFDSETTLE:
				case DAVIS640H_CONFIG_APS_GSPDRESET:
				case DAVIS640H_CONFIG_APS_GSRESETFALL:
				case DAVIS640H_CONFIG_APS_GSTXFALL:
				case DAVIS640H_CONFIG_APS_GSFDRESET:
					// Support for DAVIS640H extra timing parameters.
					if (IS_DAVIS640H(handle->info.chipID)) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SNAPSHOT: {
					// Use multi-command VR for more efficient implementation of snapshot,
					// that also guarantees returning to the default state (not running).
					if (param) {
						struct spi_config_params spiMultiConfig[2];

						spiMultiConfig[0].moduleAddr = DAVIS_CONFIG_APS;
						spiMultiConfig[0].paramAddr  = DAVIS_CONFIG_APS_RUN;
						spiMultiConfig[0].param      = true;

						spiMultiConfig[1].moduleAddr = DAVIS_CONFIG_APS;
						spiMultiConfig[1].paramAddr  = DAVIS_CONFIG_APS_RUN;
						spiMultiConfig[1].param      = false;

						return (spiConfigSendMultiple(handle->spiConfigPtr, spiMultiConfig, 2));
					}
					break;
				}

				case DAVIS_CONFIG_APS_AUTOEXPOSURE:
					atomic_store(&state->aps.autoExposure.enabled, param);
					break;

				case DAVIS_CONFIG_APS_FRAME_MODE:
					atomic_store(&state->aps.frame.mode, U8T(param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN_ACCELEROMETER:
				case DAVIS_CONFIG_IMU_RUN_GYROSCOPE:
				case DAVIS_CONFIG_IMU_RUN_TEMPERATURE:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_ACCEL_DLPF:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				case DAVIS_CONFIG_IMU_GYRO_DLPF:
					if (handle->info.imuType == 2) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_IMU, paramAddr, param));
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

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must multiply here.
					float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
					return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, U32T(timeCC)));
					break;
				}

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must multiply here.
						float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, U32T(timeCC)));
					}
					else {
						return (false);
					}
					break;
				}

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
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
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
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						case DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE:
							// Only supported by DAVIS346 and DAVIS640 chips.
							if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
								return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						case DAVIS208_CONFIG_BIAS_RESETHIGHPASS:
						case DAVIS208_CONFIG_BIAS_REFSS:
						case DAVIS208_CONFIG_BIAS_REGBIASBP:
						case DAVIS208_CONFIG_BIAS_REFSSBN:
							// Only supported by DAVIS208 chips.
							if (IS_DAVIS208(handle->info.chipID)) {
								return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						default:
							return (false);
							break;
					}
				}
				else if (IS_DAVIS640H(handle->info.chipID)) {
					// DAVIS640H also uses the 37 branches bias generator, with different values.
					switch (paramAddr) {
						case DAVIS640H_CONFIG_BIAS_APSCAS:
						case DAVIS640H_CONFIG_BIAS_OVG1LO:
						case DAVIS640H_CONFIG_BIAS_OVG2LO:
						case DAVIS640H_CONFIG_BIAS_TX2OVG2HI:
						case DAVIS640H_CONFIG_BIAS_GND07:
						case DAVIS640H_CONFIG_BIAS_ADCTESTVOLTAGE:
						case DAVIS640H_CONFIG_BIAS_ADCREFHIGH:
						case DAVIS640H_CONFIG_BIAS_ADCREFLOW:
						case DAVIS640H_CONFIG_BIAS_IFREFRBN:
						case DAVIS640H_CONFIG_BIAS_IFTHRBN:
						case DAVIS640H_CONFIG_BIAS_LOCALBUFBN:
						case DAVIS640H_CONFIG_BIAS_PADFOLLBN:
						case DAVIS640H_CONFIG_BIAS_PIXINVBN:
						case DAVIS640H_CONFIG_BIAS_DIFFBN:
						case DAVIS640H_CONFIG_BIAS_ONBN:
						case DAVIS640H_CONFIG_BIAS_OFFBN:
						case DAVIS640H_CONFIG_BIAS_PRBP:
						case DAVIS640H_CONFIG_BIAS_PRSFBP:
						case DAVIS640H_CONFIG_BIAS_REFRBP:
						case DAVIS640H_CONFIG_BIAS_ARRAYBIASBUFFERBN:
						case DAVIS640H_CONFIG_BIAS_ARRAYLOGICBUFFERBN:
						case DAVIS640H_CONFIG_BIAS_FALLTIMEBN:
						case DAVIS640H_CONFIG_BIAS_RISETIMEBP:
						case DAVIS640H_CONFIG_BIAS_READOUTBUFBP:
						case DAVIS640H_CONFIG_BIAS_APSROSFBN:
						case DAVIS640H_CONFIG_BIAS_ADCCOMPBP:
						case DAVIS640H_CONFIG_BIAS_DACBUFBP:
						case DAVIS640H_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVIS640H_CONFIG_BIAS_AEPDBN:
						case DAVIS640H_CONFIG_BIAS_AEPUXBP:
						case DAVIS640H_CONFIG_BIAS_AEPUYBP:
						case DAVIS640H_CONFIG_BIAS_BIASBUFFER:
						case DAVIS640H_CONFIG_BIAS_SSP:
						case DAVIS640H_CONFIG_BIAS_SSN:
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
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
						return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL:
						// Only supported by DAVIS240 A/B chips.
						if (IS_DAVIS240A(handle->info.chipID) || IS_DAVIS240B(handle->info.chipID)) {
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							// Keep in sync with APS module GlobalShutter parameter.
							if (!spiConfigSend(
									handle->spiConfigPtr, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, param)) {
								return (false);
							}

							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						// Only supported by the new DAVIS chips.
						if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID)
							|| IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)
							|| IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS346_CONFIG_CHIP_TESTADC:
						// Only supported by some of the new DAVIS chips.
						if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)
							|| IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS640H_CONFIG_CHIP_ADJUSTOVG1LO:    // Also DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG.
					case DAVIS640H_CONFIG_CHIP_ADJUSTOVG2LO:    // Also DAVIS208_CONFIG_CHIP_SELECTBIASREFSS.
					case DAVIS640H_CONFIG_CHIP_ADJUSTTX2OVG2HI: // Also DAVIS208_CONFIG_CHIP_SELECTSENSE.
						// Only supported by DAVIS208 and DAVIS640H.
						if (IS_DAVIS208(handle->info.chipID) || IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS208_CONFIG_CHIP_SELECTPOSFB:
					case DAVIS208_CONFIG_CHIP_SELECTHIGHPASS:
						// Only supported by DAVIS208.
						if (IS_DAVIS208(handle->info.chipID)) {
							return (spiConfigSend(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
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

		default:
			return (false);
			break;
	}

	return (true);
}

static bool davisCommonConfigGet(davisCommonHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	davisCommonState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigGet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigGet(&state->container, paramAddr, param));
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
				case DAVIS_CONFIG_MUX_RUN_CHIP:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DAVIS_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED + 1:
				case DAVIS_CONFIG_MUX_STATISTICS_DVS_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_DVS_DROPPED + 1:
					if (handle->info.muxHasStatistics) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_MUX, paramAddr, param));
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

		case DAVIS_CONFIG_DVS:
			switch (paramAddr) {
				case DAVIS_CONFIG_DVS_RUN:
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
					return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW:
					if (handle->info.dvsHasROIFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS:
				case DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY:
					if (handle->info.dvsHasSkipFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN:
				case DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS:
				case DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE:
					if (handle->info.dvsHasPolarityFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_ROW:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_ROW + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_COLUMN:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_COLUMN + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_DROPPED:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_DROPPED + 1:
					if (handle->info.dvsHasStatistics) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_PIXELS:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_PIXELS + 1:
					if (handle->info.dvsHasStatistics && handle->info.dvsHasPixelFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD + 1:
					if (handle->info.dvsHasStatistics && handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN:
					if (handle->info.dvsHasPixelFilter) {
						*param = atomic_load(&handle->state.dvs.pixelFilterAutoTrain.autoTrainRunning);
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
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_START_COLUMN_0:
				case DAVIS_CONFIG_APS_END_COLUMN_0:
				case DAVIS_CONFIG_APS_START_ROW_0:
				case DAVIS_CONFIG_APS_END_ROW_0:
					return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Use stored value.
					*param = state->aps.autoExposure.lastSetExposure;
					break;

				case DAVIS_CONFIG_APS_FRAME_INTERVAL: {
					// Frame Interval is in µs, must be converted from native FPGA cycles
					// by dividing with ADC clock value.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, &cyclesValue)) {
						return (false);
					}

					float intervalCC = roundf((float) cyclesValue / state->deviceClocks.adcClockActual);
					*param           = U32T(intervalCC);

					return (true);
					break;
				}

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS640H_CONFIG_APS_TRANSFER:
				case DAVIS640H_CONFIG_APS_RSFDSETTLE:
				case DAVIS640H_CONFIG_APS_GSPDRESET:
				case DAVIS640H_CONFIG_APS_GSRESETFALL:
				case DAVIS640H_CONFIG_APS_GSTXFALL:
				case DAVIS640H_CONFIG_APS_GSFDRESET:
					// Support for DAVIS640H extra timing parameters.
					if (IS_DAVIS640H(handle->info.chipID)) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_APS, paramAddr, param));
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
					*param = atomic_load(&state->aps.autoExposure.enabled);
					break;

				case DAVIS_CONFIG_APS_FRAME_MODE:
					*param = atomic_load(&state->aps.frame.mode);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN_ACCELEROMETER:
				case DAVIS_CONFIG_IMU_RUN_GYROSCOPE:
				case DAVIS_CONFIG_IMU_RUN_TEMPERATURE:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_ACCEL_DLPF:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				case DAVIS_CONFIG_IMU_GYRO_DLPF:
					if (handle->info.imuType == 2) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_IMU, paramAddr, param));
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

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must divide here.
						uint32_t cyclesValue = 0;
						if (!spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_EXTINPUT, paramAddr, &cyclesValue)) {
							return (false);
						}

						float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
						*param        = U32T(delayCC);

						return (true);
					}
					else {
						return (false);
					}
					break;
				}

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
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
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
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							break;

						case DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE:
							// Only supported by DAVIS346 and DAVIS640 chips.
							if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)) {
								return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						case DAVIS208_CONFIG_BIAS_RESETHIGHPASS:
						case DAVIS208_CONFIG_BIAS_REFSS:
						case DAVIS208_CONFIG_BIAS_REGBIASBP:
						case DAVIS208_CONFIG_BIAS_REFSSBN:
							// Only supported by DAVIS208 chips.
							if (IS_DAVIS208(handle->info.chipID)) {
								return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
							}
							break;

						default:
							return (false);
							break;
					}
				}
				else if (IS_DAVIS640H(handle->info.chipID)) {
					// DAVIS640H also uses the 37 branches bias generator, with different values.
					switch (paramAddr) {
						case DAVIS640H_CONFIG_BIAS_APSCAS:
						case DAVIS640H_CONFIG_BIAS_OVG1LO:
						case DAVIS640H_CONFIG_BIAS_OVG2LO:
						case DAVIS640H_CONFIG_BIAS_TX2OVG2HI:
						case DAVIS640H_CONFIG_BIAS_GND07:
						case DAVIS640H_CONFIG_BIAS_ADCTESTVOLTAGE:
						case DAVIS640H_CONFIG_BIAS_ADCREFHIGH:
						case DAVIS640H_CONFIG_BIAS_ADCREFLOW:
						case DAVIS640H_CONFIG_BIAS_IFREFRBN:
						case DAVIS640H_CONFIG_BIAS_IFTHRBN:
						case DAVIS640H_CONFIG_BIAS_LOCALBUFBN:
						case DAVIS640H_CONFIG_BIAS_PADFOLLBN:
						case DAVIS640H_CONFIG_BIAS_PIXINVBN:
						case DAVIS640H_CONFIG_BIAS_DIFFBN:
						case DAVIS640H_CONFIG_BIAS_ONBN:
						case DAVIS640H_CONFIG_BIAS_OFFBN:
						case DAVIS640H_CONFIG_BIAS_PRBP:
						case DAVIS640H_CONFIG_BIAS_PRSFBP:
						case DAVIS640H_CONFIG_BIAS_REFRBP:
						case DAVIS640H_CONFIG_BIAS_ARRAYBIASBUFFERBN:
						case DAVIS640H_CONFIG_BIAS_ARRAYLOGICBUFFERBN:
						case DAVIS640H_CONFIG_BIAS_FALLTIMEBN:
						case DAVIS640H_CONFIG_BIAS_RISETIMEBP:
						case DAVIS640H_CONFIG_BIAS_READOUTBUFBP:
						case DAVIS640H_CONFIG_BIAS_APSROSFBN:
						case DAVIS640H_CONFIG_BIAS_ADCCOMPBP:
						case DAVIS640H_CONFIG_BIAS_DACBUFBP:
						case DAVIS640H_CONFIG_BIAS_LCOLTIMEOUTBN:
						case DAVIS640H_CONFIG_BIAS_AEPDBN:
						case DAVIS640H_CONFIG_BIAS_AEPUXBP:
						case DAVIS640H_CONFIG_BIAS_AEPUYBP:
						case DAVIS640H_CONFIG_BIAS_BIASBUFFER:
						case DAVIS640H_CONFIG_BIAS_SSP:
						case DAVIS640H_CONFIG_BIAS_SSN:
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_BIAS, paramAddr, param));
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
						return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS240_CONFIG_CHIP_SPECIALPIXELCONTROL:
						// Only supported by DAVIS240 A/B chips.
						if (IS_DAVIS240A(handle->info.chipID) || IS_DAVIS240B(handle->info.chipID)) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						// Only supported by the new DAVIS chips.
						if (IS_DAVIS128(handle->info.chipID) || IS_DAVIS208(handle->info.chipID)
							|| IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)
							|| IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS346_CONFIG_CHIP_TESTADC:
						// Only supported by some of the new DAVIS chips.
						if (IS_DAVIS346(handle->info.chipID) || IS_DAVIS640(handle->info.chipID)
							|| IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS640H_CONFIG_CHIP_ADJUSTOVG1LO:    // Also DAVIS208_CONFIG_CHIP_SELECTPREAMPAVG.
					case DAVIS640H_CONFIG_CHIP_ADJUSTOVG2LO:    // Also DAVIS208_CONFIG_CHIP_SELECTBIASREFSS.
					case DAVIS640H_CONFIG_CHIP_ADJUSTTX2OVG2HI: // Also DAVIS208_CONFIG_CHIP_SELECTSENSE.
						// Only supported by DAVIS208 and DAVIS640H.
						if (IS_DAVIS208(handle->info.chipID) || IS_DAVIS640H(handle->info.chipID)) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					case DAVIS208_CONFIG_CHIP_SELECTPOSFB:
					case DAVIS208_CONFIG_CHIP_SELECTHIGHPASS:
						// Only supported by DAVIS208.
						if (IS_DAVIS208(handle->info.chipID)) {
							return (spiConfigReceive(handle->spiConfigPtr, DAVIS_CONFIG_CHIP, paramAddr, param));
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
			// Disallow system info parameters being read directly. Go via info struct!
			return (false);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

static bool davisCommonDataStart(davisCommonHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr) {
	davisCommonState state = &handle->state;

	// Store new data available/not available anymore call-backs.
	dataExchangeSetNotify(&state->dataExchange, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr);

	containerGenerationCommitTimestampReset(&state->container);

	if (!dataExchangeBufferInit(&state->dataExchange)) {
		davisLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	if (!containerGenerationAllocate(&state->container, DAVIS_EVENT_TYPES)) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPackets.polarity
		= caerPolarityEventPacketAllocate(DAVIS_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.polarity == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentPackets.special
		= caerSpecialEventPacketAllocate(DAVIS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.special == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	state->currentPackets.frame = caerFrameEventPacketAllocate(DAVIS_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), 0,
		handle->info.apsSizeX, handle->info.apsSizeY, (handle->info.apsColorFilter == MONO) ? (GRAYSCALE) : (RGB));
	if (state->currentPackets.frame == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
		return (false);
	}

	state->currentPackets.imu6 = caerIMU6EventPacketAllocate(DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.imu6 == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
		return (false);
	}

	size_t pixelsSize = sizeof(uint16_t) * (size_t) state->aps.sizeX * (size_t) state->aps.sizeY;
	// '- sizeof(uint16_t)' to compensate for pixels[1] at end of struct for C++ compatibility.
	size_t frameSize = (sizeof(struct caer_frame_event) - sizeof(uint16_t)) + pixelsSize;

	state->aps.frame.currentEvent = calloc(1, frameSize);
	if (state->aps.frame.currentEvent == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS current event memory.");
		return (false);
	}

	// Initialize constant frame data.
	caerFrameEventSetColorFilter(state->aps.frame.currentEvent, handle->info.apsColorFilter);
	caerFrameEventSetROIIdentifier(state->aps.frame.currentEvent, 0);

#if APS_DEBUG_FRAME == 1
	state->aps.frame.resetPixels = calloc((size_t)(state->aps.sizeX * state->aps.sizeY), sizeof(uint16_t));
	if (state->aps.frame.resetPixels == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS reset pixels memory (debug).");
		return (false);
	}

	state->aps.frame.signalPixels = calloc((size_t)(state->aps.sizeX * state->aps.sizeY), sizeof(uint16_t));
	if (state->aps.frame.signalPixels == NULL) {
		freeAllDataMemory(state);

		davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS signal pixels memory (debug).");
		return (false);
	}
#endif

	// Ignore multi-part events (APS and IMU) at startup, so that any initial
	// incomplete event is ignored. The START events reset this as soon as
	// the first one is observed.
	state->aps.ignoreEvents = true;
	state->imu.ignoreEvents = true;

	return (true);
}

static void davisCommonDataStop(davisCommonHandle handle) {
	davisCommonState state = &handle->state;

	dataExchangeBufferEmpty(&state->dataExchange);

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPackets.polarityPosition = 0;
	state->currentPackets.specialPosition  = 0;
	state->currentPackets.framePosition    = 0;
	state->currentPackets.imu6Position     = 0;

	// Reset private composite events. 'aps.currentEvent' is taken care of in freeAllDataMemory().
	memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));
}

#define TS_WRAP_ADD 0x8000

static void davisCommonEventTranslator(
	davisCommonHandle handle, const uint8_t *buffer, size_t bufferSize, atomic_uint_fast32_t *transfersRunning) {
	davisCommonState state = &handle->state;

	// Truncate off any extra partial event.
	if ((bufferSize & 0x01) != 0) {
		davisLog(CAER_LOG_ALERT, handle, "%zu bytes received, which is not a multiple of two.", bufferSize);
		bufferSize &= ~((size_t) 0x01);
	}

	for (size_t bufferPos = 0; bufferPos < bufferSize; bufferPos += 2) {
		// Allocate new packets for next iteration as needed.
		if (!containerGenerationAllocate(&state->container, DAVIS_EVENT_TYPES)) {
			davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
			return;
		}

		if (state->currentPackets.special == NULL) {
			state->currentPackets.special = caerSpecialEventPacketAllocate(
				DAVIS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.special == NULL) {
				davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}

		if (state->currentPackets.polarity == NULL) {
			state->currentPackets.polarity = caerPolarityEventPacketAllocate(
				DAVIS_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.polarity == NULL) {
				davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}

		if (state->currentPackets.frame == NULL) {
			state->currentPackets.frame = caerFrameEventPacketAllocate(DAVIS_FRAME_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->timestamps.wrapOverflow, handle->info.apsSizeX,
				handle->info.apsSizeY, (handle->info.apsColorFilter == MONO) ? (GRAYSCALE) : (RGB));
			if (state->currentPackets.frame == NULL) {
				davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
				return;
			}
		}

		if (state->currentPackets.imu6 == NULL) {
			state->currentPackets.imu6 = caerIMU6EventPacketAllocate(
				DAVIS_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.imu6 == NULL) {
				davisLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
				return;
			}
		}

		bool tsReset   = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(*((const uint16_t *) (&buffer[bufferPos])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			handleTimestampUpdateNewLogic(&state->timestamps, event, handle->info.deviceString, &state->deviceLogLevel);

			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code  = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							davisLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							handleTimestampResetNewLogic(
								&state->timestamps, handle->info.deviceString, &state->deviceLogLevel);

							containerGenerationCommitTimestampReset(&state->container);
							containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets.
							// Async call to not deadlock here.
							spiConfigReceiveAsync(handle->spiConfigPtr, DAVIS_CONFIG_SYSINFO,
								DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER, &davisCommonTSMasterStatusUpdater,
								&handle->info);
							break;
						}

						case 2: { // External input (falling edge)
							davisLog(CAER_LOG_DEBUG, handle, "External input (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 3: { // External input (rising edge)
							davisLog(CAER_LOG_DEBUG, handle, "External input (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 4: { // External input (pulse)
							davisLog(CAER_LOG_DEBUG, handle, "External input (pulse) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 5: { // IMU Start (6 axes)
							davisLog(CAER_LOG_DEBUG, handle, "IMU6 Start event received.");

							state->imu.ignoreEvents = false;
							state->imu.count        = 0;
							state->imu.type         = 0;

							memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

							break;
						}

						case 7: { // IMU End
							if (state->imu.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "IMU End event received.");

							if (state->imu.count == IMU_TOTAL_COUNT) {
								// Timestamp at event-stream insertion point.
								caerIMU6EventSetTimestamp(&state->imu.currentEvent, state->timestamps.current);

								caerIMU6EventValidate(&state->imu.currentEvent, state->currentPackets.imu6);

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
								if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.imu6,
										(size_t) state->currentPackets.imu6Position, 1, handle)) {
									caerIMU6Event imuCurrentEvent = caerIMU6EventPacketGetEvent(
										state->currentPackets.imu6, state->currentPackets.imu6Position);
									memcpy(imuCurrentEvent, &state->imu.currentEvent, sizeof(struct caer_imu6_event));
									state->currentPackets.imu6Position++;
								}
							}
							else {
								davisLog(CAER_LOG_INFO, handle,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imu.count);
							}
							break;
						}

						case 8: { // APS Global Shutter Frame Start
							davisLog(CAER_LOG_DEBUG, handle, "APS GS Frame Start event received.");
							state->aps.globalShutter = true;

							apsInitFrame(handle);

							break;
						}

						case 9: { // APS Rolling Shutter Frame Start
							davisLog(CAER_LOG_DEBUG, handle, "APS RS Frame Start event received.");
							state->aps.globalShutter = false;

							apsInitFrame(handle);

							break;
						}

						case 10: { // APS Frame End
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Frame End event received.");

							// NOTE: IMU6 and APS operate on an internal event and copy that to the actual output
							// packet here, in the END state, for a reason: if a packetContainer, with all its
							// packets, is committed due to hitting any of the triggers that are not TS reset
							// or TS wrap-around related, like number of polarity events, the event in the packet
							// would be left incomplete, and the event in the new packet would be corrupted.
							// We could avoid this like for the TS reset/TS wrap-around case (see tsReset) by
							// just deleting that event, but these kinds of commits happen much more often and the
							// possible data loss would be too significant. So instead we keep a private event,
							// fill it, and then only copy it into the packet here in the END state, at which point
							// the whole event is ready and cannot be broken/corrupted in any way anymore.
							bool validFrame = apsEndFrame(handle);

							// Validate event and advance frame packet position.
							if (validFrame) {
								// Get next frame.
								if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.frame,
										(size_t) state->currentPackets.framePosition, 1, handle)) {
									caerFrameEvent frameEvent = caerFrameEventPacketGetEvent(
										state->currentPackets.frame, state->currentPackets.framePosition);
									state->currentPackets.framePosition++;

									// Finalize frame setup.
									caerFrameEventSetPositionX(state->aps.frame.currentEvent, state->aps.roi.positionX);
									caerFrameEventSetPositionY(state->aps.frame.currentEvent, state->aps.roi.positionY);
									caerFrameEventSetLengthXLengthYChannelNumber(state->aps.frame.currentEvent,
										state->aps.roi.sizeX, state->aps.roi.sizeY, GRAYSCALE,
										state->currentPackets.frame);

									// Automatic exposure control support.
									if (atomic_load_explicit(&state->aps.autoExposure.enabled, memory_order_relaxed)) {
										float exposureFrameCC
											= roundf((float) state->aps.autoExposure.currentFrameExposure
													 / state->deviceClocks.adcClockActual);

										int32_t newExposureValue = autoExposureCalculate(&state->aps.autoExposure.state,
											state->aps.frame.currentEvent, U32T(exposureFrameCC),
											state->aps.autoExposure.lastSetExposure,
											atomic_load_explicit(&state->deviceLogLevel, memory_order_relaxed),
											handle->info.deviceString);

										if (newExposureValue >= 0) {
											// Update exposure value. Done in main thread to avoid deadlock inside
											// callback.
											davisLog(CAER_LOG_DEBUG, handle,
												"Automatic exposure control set exposure to %" PRIi32 " µs.",
												newExposureValue);

											state->aps.autoExposure.lastSetExposure = U32T(newExposureValue);

											float newExposureCC
												= roundf((float) newExposureValue * state->deviceClocks.adcClockActual);

											spiConfigSendAsync(handle->spiConfigPtr, DAVIS_CONFIG_APS,
												DAVIS_CONFIG_APS_EXPOSURE, U32T(newExposureCC), NULL, NULL);
										}
									}

									// Copy header over.
									memcpy(frameEvent, state->aps.frame.currentEvent,
										(sizeof(struct caer_frame_event) - sizeof(uint16_t)));

									if (handle->info.apsColorFilter != MONO) {
										// Color camera. Frame mode decides what to return.
										enum caer_davis_aps_frame_modes frameMode
											= atomic_load_explicit(&state->aps.frame.mode, memory_order_relaxed);

										if (frameMode == APS_FRAME_DEFAULT) {
											// Default for color sensor means a color image.
											// Set destination to RGB and do interpolation.
											caerFrameEventSetLengthXLengthYChannelNumber(frameEvent,
												state->aps.roi.sizeX, state->aps.roi.sizeY, RGB,
												state->currentPackets.frame);

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
											caerFrameUtilsDemosaic(
												state->aps.frame.currentEvent, frameEvent, DEMOSAIC_OPENCV_STANDARD);
#else
											caerFrameUtilsDemosaic(
												state->aps.frame.currentEvent, frameEvent, DEMOSAIC_STANDARD);
#endif
										}
										else if (frameMode == APS_FRAME_GRAYSCALE) {
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
											caerFrameUtilsDemosaic(
												state->aps.frame.currentEvent, frameEvent, DEMOSAIC_OPENCV_TO_GRAY);
#else
											caerFrameUtilsDemosaic(
												state->aps.frame.currentEvent, frameEvent, DEMOSAIC_TO_GRAY);
#endif
										}
										else {
											// APS_FRAME_ORIGINAL, just copy pixels.
											memcpy(caerFrameEventGetPixelArrayUnsafe(frameEvent),
												caerFrameEventGetPixelArrayUnsafeConst(state->aps.frame.currentEvent),
												caerFrameEventGetPixelsSize(state->aps.frame.currentEvent));
										}
									}
									else {
										// Grayscale camera. All modes are equal here
										// Header is already grayscale and fully setup.
										// Always copy the pixels over and that's it.
										memcpy(caerFrameEventGetPixelArrayUnsafe(frameEvent),
											caerFrameEventGetPixelArrayUnsafeConst(state->aps.frame.currentEvent),
											caerFrameEventGetPixelsSize(state->aps.frame.currentEvent));
									}

									// Finally, validate new frame.
									caerFrameEventValidate(frameEvent, state->currentPackets.frame);
								}

// Separate debug support.
#if APS_DEBUG_FRAME == 1
								// Get debug frames.
								if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.frame,
										(size_t) state->currentPackets.framePosition, 2, handle)) {
									// Reset frame.
									caerFrameEvent resetFrameEvent = caerFrameEventPacketGetEvent(
										state->currentPackets.frame, state->currentPackets.framePosition);
									state->currentPackets.framePosition++;

									// Setup new frame.
									caerFrameEventSetColorFilter(resetFrameEvent, handle->info.apsColorFilter);
									caerFrameEventSetROIIdentifier(resetFrameEvent, 1);
									caerFrameEventSetTSStartOfFrame(resetFrameEvent,
										caerFrameEventGetTSStartOfFrame(state->aps.frame.currentEvent));
									caerFrameEventSetTSStartOfExposure(resetFrameEvent,
										caerFrameEventGetTSStartOfExposure(state->aps.frame.currentEvent));
									caerFrameEventSetTSEndOfExposure(resetFrameEvent,
										caerFrameEventGetTSEndOfExposure(state->aps.frame.currentEvent));
									caerFrameEventSetTSEndOfFrame(
										resetFrameEvent, caerFrameEventGetTSEndOfFrame(state->aps.frame.currentEvent));
									caerFrameEventSetPositionX(resetFrameEvent, state->aps.roi.positionX);
									caerFrameEventSetPositionY(resetFrameEvent, state->aps.roi.positionY);
									caerFrameEventSetLengthXLengthYChannelNumber(resetFrameEvent, state->aps.roi.sizeX,
										state->aps.roi.sizeY, GRAYSCALE, state->currentPackets.frame);
									caerFrameEventValidate(resetFrameEvent, state->currentPackets.frame);

									// Copy pixels over.
									memcpy(caerFrameEventGetPixelArrayUnsafe(resetFrameEvent),
										state->aps.frame.resetPixels, caerFrameEventGetPixelsSize(resetFrameEvent));

									// Signal frame.
									caerFrameEvent signalFrameEvent = caerFrameEventPacketGetEvent(
										state->currentPackets.frame, state->currentPackets.framePosition);
									state->currentPackets.framePosition++;

									// Setup new frame.
									caerFrameEventSetColorFilter(signalFrameEvent, handle->info.apsColorFilter);
									caerFrameEventSetROIIdentifier(signalFrameEvent, 2);
									caerFrameEventSetTSStartOfFrame(resetFrameEvent,
										caerFrameEventGetTSStartOfFrame(state->aps.frame.currentEvent));
									caerFrameEventSetTSStartOfExposure(resetFrameEvent,
										caerFrameEventGetTSStartOfExposure(state->aps.frame.currentEvent));
									caerFrameEventSetTSEndOfExposure(resetFrameEvent,
										caerFrameEventGetTSEndOfExposure(state->aps.frame.currentEvent));
									caerFrameEventSetTSEndOfFrame(
										resetFrameEvent, caerFrameEventGetTSEndOfFrame(state->aps.frame.currentEvent));
									caerFrameEventSetPositionX(signalFrameEvent, state->aps.roi.positionX);
									caerFrameEventSetPositionY(signalFrameEvent, state->aps.roi.positionY);
									caerFrameEventSetLengthXLengthYChannelNumber(signalFrameEvent, state->aps.roi.sizeX,
										state->aps.roi.sizeY, GRAYSCALE, state->currentPackets.frame);
									caerFrameEventValidate(signalFrameEvent, state->currentPackets.frame);

									// Copy pixels over.
									memcpy(caerFrameEventGetPixelArrayUnsafe(signalFrameEvent),
										state->aps.frame.signalPixels, caerFrameEventGetPixelsSize(signalFrameEvent));
								}
#endif
							}

							break;
						}

						case 11: { // APS Reset Column Start
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Reset Column Start event received.");

							state->aps.currentReadoutType        = APS_READOUT_RESET;
							state->aps.countY[APS_READOUT_RESET] = 0;

							state->aps.cDavisSupport.offsetDirection = 0;
							state->aps.cDavisSupport.offset          = 1; // First pixel of row always even.

							break;
						}

						case 12: { // APS Signal Column Start
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Signal Column Start event received.");

							state->aps.currentReadoutType         = APS_READOUT_SIGNAL;
							state->aps.countY[APS_READOUT_SIGNAL] = 0;

							state->aps.cDavisSupport.offsetDirection = 0;
							state->aps.cDavisSupport.offset          = 1; // First pixel of row always even.

							break;
						}

						case 13: { // APS Column End
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Column End event received.");

							davisLog(CAER_LOG_DEBUG, handle, "APS Column End: CountX[%d] is %d.",
								state->aps.currentReadoutType, state->aps.countX[state->aps.currentReadoutType]);
							davisLog(CAER_LOG_DEBUG, handle, "APS Column End: CountY[%d] is %d.",
								state->aps.currentReadoutType, state->aps.countY[state->aps.currentReadoutType]);

							if (state->aps.countY[state->aps.currentReadoutType] != state->aps.expectedCountY) {
								davisLog(CAER_LOG_ERROR, handle,
									"APS Column End - %d - %d: wrong row count %d detected, expected %d.",
									state->aps.currentReadoutType, state->aps.countX[state->aps.currentReadoutType],
									state->aps.countY[state->aps.currentReadoutType], state->aps.expectedCountY);
							}

							state->aps.countX[state->aps.currentReadoutType]++;

							break;
						}

						case 14: { // APS Exposure Start
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Exposure Start event received.");

							caerFrameEventSetTSStartOfExposure(
								state->aps.frame.currentEvent, state->timestamps.current);

							// Send APS info event out (as special event).
							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 15: { // APS Exposure End
							if (state->aps.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "APS Exposure End event received.");

							caerFrameEventSetTSEndOfExposure(state->aps.frame.currentEvent, state->timestamps.current);

							// Send APS info event out (as special event).
							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_END);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 16: { // External generator (falling edge)
							davisLog(CAER_LOG_DEBUG, handle, "External generator (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 17: { // External generator (rising edge)
							davisLog(CAER_LOG_DEBUG, handle, "External generator (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						default:
							davisLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.", data);
							break;
					}
					break;

				case 1: // Y address
					// Check range conformity.
					if (data >= state->dvs.sizeY) {
						davisLog(CAER_LOG_ALERT, handle, "DVS: Y address out of range (0-%d): %" PRIu16 ".",
							state->dvs.sizeY - 1, data);
						break; // Skip invalid Y address (don't update lastY).
					}

					state->dvs.lastY = data;

					break;

				case 2:   // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= state->dvs.sizeX) {
						davisLog(CAER_LOG_ALERT, handle, "DVS: X address out of range (0-%d): %" PRIu16 ".",
							state->dvs.sizeX - 1, data);
						break; // Skip invalid event.
					}

					// Invert polarity for PixelParade high gain pixels (DavisSense), because of negative gain from
					// pre-amplifier. uint8_t polarity = ((IS_DAVIS208(handle->info.chipID)) && (data < 192)) ?
					// U8T(~code) : (code);

					if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.polarity,
							(size_t) state->currentPackets.polarityPosition, 1, handle)) {
						caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
							state->currentPackets.polarity, state->currentPackets.polarityPosition);

						// Timestamp at event-stream insertion point.
						caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
						caerPolarityEventSetPolarity(currentPolarityEvent, (code & 0x01));
						if (state->dvs.invertXY) {
							caerPolarityEventSetY(currentPolarityEvent, data);
							caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastY);
						}
						else {
							caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastY);
							caerPolarityEventSetX(currentPolarityEvent, data);
						}
						caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
						state->currentPackets.polarityPosition++;
					}

					break;
				}

				case 4: {
					if (state->aps.ignoreEvents) {
						break;
					}

					// Ignore too big X/Y counts, can happen if column start/end events are lost.
					if ((state->aps.countX[state->aps.currentReadoutType] >= state->aps.expectedCountX)
						|| (state->aps.countY[state->aps.currentReadoutType] >= state->aps.expectedCountY)) {
						break;
					}

					// DAVIS240 has a reduced dynamic range due to external
					// ADC high/low ref resistors not having optimal values.
					// To fix this multiply by 1.95 to 2.15, so we choose to
					// just shift by one (multiply by 2.00) for efficiency.
					if (IS_DAVIS240(handle->info.chipID)) {
						data = U16T(data << 1);
					}

					apsUpdateFrame(handle, data);

					state->aps.countY[state->aps.currentReadoutType]++;

					break;
				}

				case 5: {
					// Misc 8bit data.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imu.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "IMU Data event (%" PRIu8 ") received.", misc8Data);

							// IMU data event.
							switch (state->imu.count) {
								case 0:
								case 2:
								case 4:
								case 6:
								case 8:
								case 10:
								case 12:
									state->imu.tmpData = misc8Data;
									break;

								case 1: {
									int16_t accelX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										accelX = I16T(-accelX);
									}
									caerIMU6EventSetAccelX(&state->imu.currentEvent, accelX / state->imu.accelScale);
									break;
								}

								case 3: {
									int16_t accelY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										accelY = I16T(-accelY);
									}
									caerIMU6EventSetAccelY(&state->imu.currentEvent, accelY / state->imu.accelScale);
									break;
								}

								case 5: {
									int16_t accelZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										accelZ = I16T(-accelZ);
									}
									caerIMU6EventSetAccelZ(&state->imu.currentEvent, accelZ / state->imu.accelScale);

									// IMU parser count depends on which data is present.
									if (!(state->imu.type & IMU_TYPE_TEMP)) {
										if (state->imu.type & IMU_TYPE_GYRO) {
											// No temperature, but gyro.
											state->imu.count = U8T(state->imu.count + 2);
										}
										else {
											// No others enabled.
											state->imu.count = U8T(state->imu.count + 8);
										}
									}
									break;
								}

								case 7: {
									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 340) + 35 for InvenSense MPU-6050
									// (SIGNED_VAL / 333.87) + 21 for InvenSense MPU-9250
									int16_t temp = I16T((state->imu.tmpData << 8) | misc8Data);

									if (handle->info.imuType == IMU_INVENSENSE_9250) {
										caerIMU6EventSetTemp(&state->imu.currentEvent, (temp / 333.87F) + 21.0F);
									}
									else {
										caerIMU6EventSetTemp(&state->imu.currentEvent, (temp / 340.0F) + 35.0F);
									}

									// IMU parser count depends on which data is present.
									if (!(state->imu.type & IMU_TYPE_GYRO)) {
										// No others enabled.
										state->imu.count = U8T(state->imu.count + 6);
									}
									break;
								}

								case 9: {
									int16_t gyroX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										gyroX = I16T(-gyroX);
									}
									caerIMU6EventSetGyroX(&state->imu.currentEvent, gyroX / state->imu.gyroScale);
									break;
								}

								case 11: {
									int16_t gyroY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										gyroY = I16T(-gyroY);
									}
									caerIMU6EventSetGyroY(&state->imu.currentEvent, gyroY / state->imu.gyroScale);
									break;
								}

								case 13: {
									int16_t gyroZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										gyroZ = I16T(-gyroZ);
									}
									caerIMU6EventSetGyroZ(&state->imu.currentEvent, gyroZ / state->imu.gyroScale);
									break;
								}

								default:
									davisLog(CAER_LOG_ERROR, handle, "Got invalid IMU update sequence.");
									break;
							}

							state->imu.count++;

							break;

						case 1:
							// APS ROI Size Part 1 (bits 15-8).
							// Here we just store the temporary value, and use it again
							// in the next case statement.
							state->aps.roi.tmpData = U16T(misc8Data << 8);

							break;

						case 2: {
							// APS ROI Size Part 2 (bits 7-0).
							// Here we just store the values and re-use the four fields
							// sizeX/Y and positionX/Y to store endCol/Row and startCol/Row.
							// We then recalculate all the right values and set everything
							// up in START_FRAME.
							switch (state->aps.roi.update & 0x03) {
								case 0:
									// START COLUMN
									state->aps.roi.positionX = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 1:
									// START ROW
									state->aps.roi.positionY = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 2:
									// END COLUMN
									state->aps.roi.sizeX = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 3:
									// END ROW
									state->aps.roi.sizeY = U16T(state->aps.roi.tmpData | misc8Data);

									// Got all sizes, now compute correct window.
									apsROIUpdateSizes(handle);
									break;

								default:
									davisLog(CAER_LOG_ERROR, handle, "Got invalid ROI update sequence.");
									break;
							}

							// Jump to next type of APS info (col->row, start->end).
							state->aps.roi.update++;

							break;
						}

						case 3: {
							if (state->imu.ignoreEvents) {
								break;
							}
							davisLog(CAER_LOG_DEBUG, handle, "IMU Scale Config event (%" PRIu16 ") received.", data);

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imu.accelScale = calculateIMUAccelScale(U16T(data >> 2) & 0x03);
							state->imu.gyroScale  = calculateIMUGyroScale(data & 0x03);

							// Set expected type of data to come from IMU (accel, gyro, temp).
							state->imu.type = (data >> 5) & 0x07;

							// IMU parser start count depends on which data is present.
							if (state->imu.type & IMU_TYPE_ACCEL) {
								// Accelerometer.
								state->imu.count = 0;
							}
							else if (state->imu.type & IMU_TYPE_TEMP) {
								// Temperature
								state->imu.count = 6;
							}
							else if (state->imu.type & IMU_TYPE_GYRO) {
								// Gyroscope.
								state->imu.count = 8;
							}
							else {
								// Nothing, should never happen.
								state->imu.count = 14;

								davisLog(CAER_LOG_ERROR, handle, "IMU Scale Config: no IMU sensors enabled.");
							}

							break;
						}

						default:
							davisLog(CAER_LOG_ERROR, handle, "Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 6: {
					// Misc 10bit data.
					uint8_t misc10Code  = U8T((data & 0x0C00) >> 10);
					uint16_t misc10Data = U16T(data & 0x03FF);

					switch (misc10Code) {
						case 0:
							state->aps.autoExposure.currentFrameExposure
								|= (U32T(misc10Data) << U32T(10 * state->aps.autoExposure.tmpData));
							state->aps.autoExposure.tmpData++;
							break;

						default:
							davisLog(CAER_LOG_ERROR, handle, "Caught Misc10 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: { // Timestamp wrap
					tsBigWrap = handleTimestampWrapNewLogic(
						&state->timestamps, data, TS_WRAP_ADD, handle->info.deviceString, &state->deviceLogLevel);

					if (tsBigWrap) {
						if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
								(size_t) state->currentPackets.specialPosition, 1, handle)) {
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
							caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
						}
					}
					else {
						containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
					}

					break;
				}

				default:
					davisLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// tsReset and tsBigWrap are already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = containerGenerationGetMaxPacketSize(&state->container);
		bool containerSizeCommit                 = (currentPacketContainerCommitSize > 0)
								   && ((state->currentPackets.polarityPosition >= currentPacketContainerCommitSize)
										  || (state->currentPackets.specialPosition >= currentPacketContainerCommitSize)
										  || (state->currentPackets.framePosition >= currentPacketContainerCommitSize)
										  || (state->currentPackets.imu6Position >= currentPacketContainerCommitSize));

		bool containerTimeCommit = containerGenerationIsCommitTimestampElapsed(
			&state->container, state->timestamps.wrapOverflow, state->timestamps.current);

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPackets.polarityPosition > 0) {
				containerGenerationSetPacket(
					&state->container, POLARITY_EVENT, (caerEventPacketHeader) state->currentPackets.polarity);

				// Run pixel filter auto-train. Can only be enabled if hw-filter present.
				if (atomic_load_explicit(&state->dvs.pixelFilterAutoTrain.autoTrainRunning, memory_order_relaxed)) {
					if (state->dvs.pixelFilterAutoTrain.noiseFilter == NULL) {
						state->dvs.pixelFilterAutoTrain.noiseFilter
							= caerFilterDVSNoiseInitialize(U16T(handle->info.dvsSizeX), U16T(handle->info.dvsSizeY));
						if (state->dvs.pixelFilterAutoTrain.noiseFilter == NULL) {
							// Failed to initialize, auto-training not possible.
							atomic_store(&state->dvs.pixelFilterAutoTrain.autoTrainRunning, false);
							goto out;
						}

						// Allocate+init success, configure it for hot-pixel learning.
						caerFilterDVSNoiseConfigSet(
							state->dvs.pixelFilterAutoTrain.noiseFilter, CAER_FILTER_DVS_HOTPIXEL_COUNT, 1000);
						caerFilterDVSNoiseConfigSet(
							state->dvs.pixelFilterAutoTrain.noiseFilter, CAER_FILTER_DVS_HOTPIXEL_TIME, 1000000);
						caerFilterDVSNoiseConfigSet(
							state->dvs.pixelFilterAutoTrain.noiseFilter, CAER_FILTER_DVS_HOTPIXEL_LEARN, true);
					}

					// NoiseFilter must be allocated and initialized if we get here.
					caerFilterDVSNoiseApply(
						state->dvs.pixelFilterAutoTrain.noiseFilter, state->currentPackets.polarity);

					uint64_t stillLearning = 1;
					caerFilterDVSNoiseConfigGet(
						state->dvs.pixelFilterAutoTrain.noiseFilter, CAER_FILTER_DVS_HOTPIXEL_LEARN, &stillLearning);

					if (!stillLearning) {
						// Learning done, we can grab the list of hot pixels, and hardware-filter them.
						caerFilterDVSPixel hotPixels;
						ssize_t hotPixelsSize
							= caerFilterDVSNoiseGetHotPixels(state->dvs.pixelFilterAutoTrain.noiseFilter, &hotPixels);
						if (hotPixelsSize < 0) {
							// Failed to get list.
							atomic_store(&state->dvs.pixelFilterAutoTrain.autoTrainRunning, false);
							goto out;
						}

						// Limit to maximum hardware size.
						if (hotPixelsSize > DVS_HOTPIXEL_HW_MAX) {
							hotPixelsSize = DVS_HOTPIXEL_HW_MAX;
						}

						// Go through the found pixels and filter them. Disable not used slots.
						size_t i = 0;

						for (; i < (size_t) hotPixelsSize; i++) {
							spiConfigSendAsync(handle->spiConfigPtr, DAVIS_CONFIG_DVS,
								U8T(DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN + 2 * i),
								(state->dvs.invertXY) ? (hotPixels[i].y) : (hotPixels[i].x), NULL, NULL);
							spiConfigSendAsync(handle->spiConfigPtr, DAVIS_CONFIG_DVS,
								U8T(DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW + 2 * i),
								(state->dvs.invertXY) ? (hotPixels[i].x) : (hotPixels[i].y), NULL, NULL);
						}

						for (; i < DVS_HOTPIXEL_HW_MAX; i++) {
							spiConfigSendAsync(handle->spiConfigPtr, DAVIS_CONFIG_DVS,
								U8T(DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN + 2 * i), U32T(state->dvs.sizeX), NULL,
								NULL);
							spiConfigSendAsync(handle->spiConfigPtr, DAVIS_CONFIG_DVS,
								U8T(DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW + 2 * i), U32T(state->dvs.sizeY), NULL, NULL);
						}

						// We're done!
						free(hotPixels);

						atomic_store(&state->dvs.pixelFilterAutoTrain.autoTrainRunning, false);
						goto out;
					}
				}
				else {
				out:
					// Deallocate when turned off, either by user or by having completed.
					if (state->dvs.pixelFilterAutoTrain.noiseFilter != NULL) {
						caerFilterDVSNoiseDestroy(state->dvs.pixelFilterAutoTrain.noiseFilter);
						state->dvs.pixelFilterAutoTrain.noiseFilter = NULL;
					}
				}

				state->currentPackets.polarity         = NULL;
				state->currentPackets.polarityPosition = 0;
				emptyContainerCommit                   = false;
			}

			if (state->currentPackets.specialPosition > 0) {
				containerGenerationSetPacket(
					&state->container, SPECIAL_EVENT, (caerEventPacketHeader) state->currentPackets.special);

				state->currentPackets.special         = NULL;
				state->currentPackets.specialPosition = 0;
				emptyContainerCommit                  = false;
			}

			if (state->currentPackets.framePosition > 0) {
				containerGenerationSetPacket(
					&state->container, FRAME_EVENT, (caerEventPacketHeader) state->currentPackets.frame);

				state->currentPackets.frame         = NULL;
				state->currentPackets.framePosition = 0;
				emptyContainerCommit                = false;
			}

			if (state->currentPackets.imu6Position > 0) {
				containerGenerationSetPacket(
					&state->container, IMU6_EVENT, (caerEventPacketHeader) state->currentPackets.imu6);

				state->currentPackets.imu6         = NULL;
				state->currentPackets.imu6Position = 0;
				emptyContainerCommit               = false;
			}

			if (tsReset || tsBigWrap) {
				// Ignore all APS and IMU6 (composite) events, until a new APS or IMU6
				// Start event comes in, for the next packet.
				// This is to correctly support the forced packet commits that a TS reset,
				// or a TS big wrap, impose. Continuing to parse events would result
				// in a corrupted state of the first event in the new packet, as it would
				// be incomplete, incorrect and miss vital initialization data.
				// See APS and IMU6 END states for more details on a related issue.
				state->aps.ignoreEvents = true;
				state->imu.ignoreEvents = true;
			}

			containerGenerationExecute(&state->container, emptyContainerCommit, tsReset, state->timestamps.wrapOverflow,
				state->timestamps.current, &state->dataExchange, transfersRunning, handle->info.deviceID,
				handle->info.deviceString, &state->deviceLogLevel);
		}
	}
}

static void davisCommonTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param) {
	// If any error happened, discard.
	if (status != 0) {
		return;
	}

	// Get new Master/Slave information from device. Done here to prevent deadlock
	// inside asynchronous callback.
	struct caer_davis_info *info = userDataPtr;

	atomic_thread_fence(memory_order_seq_cst);
	info->deviceIsMaster = param;
	atomic_thread_fence(memory_order_seq_cst);
}

#endif /* LIBCAER_SRC_DAVIS_COMMON_H_ */
