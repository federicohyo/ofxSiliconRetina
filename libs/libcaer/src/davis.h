#ifndef LIBCAER_SRC_DAVIS_H_
#define LIBCAER_SRC_DAVIS_H_

#include "devices/davis.h"
#include "data_exchange.h"
#include "container_generation.h"
#include "usb_utils.h"
#include "autoexposure.h"

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET  0
#define APS_READOUT_SIGNAL 1

/**
 * Enable APS frame debugging by only looking at the reset or signal
 * frames, and not at the resulting correlated frame.
 * Supported values:
 * 0 - normal output, no debug (default)
 * 1 - both reset and signal separately (marked as ROI regions
 *     [0,1,2,3] for signal and [4,5,6,7] for reset respectively)
 */
#define APS_DEBUG_FRAME 0

#define APS_ADC_DEPTH 10

#define APS_ADC_CHANNELS 1

#define APS_ROI_REGIONS DAVIS_APS_ROI_REGIONS_MAX

#define IMU6_COUNT 15
#define IMU9_COUNT 21

#define DAVIS_EVENT_TYPES 5
#define DAVIS_SAMPLE_POSITION 4

#define DAVIS_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_FRAME_DEFAULT_SIZE 8
#define DAVIS_IMU_DEFAULT_SIZE 64
#define DAVIS_SAMPLE_DEFAULT_SIZE 512

#define DAVIS_DEVICE_NAME "DAVIS"

#define DAVIS_FX2_DEVICE_PID 0x841B
#define DAVIS_FX2_REQUIRED_LOGIC_REVISION 9912
#define DAVIS_FX2_REQUIRED_FIRMWARE_VERSION 4
#define DAVIS_FX2_USB_CLOCK_FREQ 30

#define DAVIS_FX3_DEVICE_PID 0x841A
#define DAVIS_FX3_REQUIRED_LOGIC_REVISION 9912
#define DAVIS_FX3_REQUIRED_FIRMWARE_VERSION 4
#define DAVIS_FX3_USB_CLOCK_FREQ 80
#define DAVIS_FX3_CLOCK_FREQ_CORRECTION 1.008f

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

struct davis_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastY;
		bool gotY;
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
	} dvs;
	struct {
		// APS specific fields
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
		bool flipX;
		bool flipY;
		bool ignoreEvents;
		bool globalShutter;
		bool resetRead;
		uint16_t currentReadoutType;
		uint16_t countX[APS_READOUT_TYPES_NUM];
		uint16_t countY[APS_READOUT_TYPES_NUM];
		uint16_t expectedCountX;
		uint16_t *expectedCountY;
		struct {
			int32_t tsStartFrame;
			int32_t tsStartExposure;
			int32_t tsEndExposure;
			size_t *pixelIndexes;
			size_t pixelIndexesPosition[APS_READOUT_TYPES_NUM];
			uint16_t *resetPixels;
			uint16_t *pixels;
		} frame;
		struct {
			// Temporary values from device.
			uint16_t update;
			uint16_t tmpData;
			bool deviceEnabled[APS_ROI_REGIONS];
			uint16_t startColumn[APS_ROI_REGIONS];
			uint16_t startRow[APS_ROI_REGIONS];
			uint16_t endColumn[APS_ROI_REGIONS];
			uint16_t endRow[APS_ROI_REGIONS];
			// Parameters for frame parsing.
			bool enabled[APS_ROI_REGIONS];
			uint16_t positionX[APS_ROI_REGIONS];
			uint16_t positionY[APS_ROI_REGIONS];
			uint16_t sizeX[APS_ROI_REGIONS];
			uint16_t sizeY[APS_ROI_REGIONS];
		} roi;
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
		uint8_t count;
		uint8_t tmpData;
		float accelScale;
		float gyroScale;
		// Current composite events, for later copy, to not loose them on commits.
		struct caer_imu6_event currentEvent;
	} imu;
	struct {
		// Microphone specific fields
		bool isRight;
		uint8_t count;
		uint16_t tmpData;
	} mic;
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
		// Microphone Sample Packet state
		caerSampleEventPacket sample;
		int32_t samplePosition;
	} currentPackets;
	struct {
		// Debug transfer support (FX3 only).
		bool enabled;
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct davis_state *davisState;

struct davis_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_davis_info info;
	// State for data management, common to all DAVIS.
	struct davis_state state;
};

typedef struct davis_handle *davisHandle;

caerDeviceHandle davisOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
caerDeviceHandle davisFX2Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
caerDeviceHandle davisFX3Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);

caerDeviceHandle davisOpenInternal(uint16_t deviceType, uint16_t deviceID, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict);
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
