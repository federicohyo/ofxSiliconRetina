#ifndef LIBCAER_SRC_DAVIS_COMMON_H_
#define LIBCAER_SRC_DAVIS_COMMON_H_

#include "devices/davis.h"
#include "ringbuffer/ringbuffer.h"
#include "usb_utils.h"
#include "autoexposure.h"

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET  0
#define APS_READOUT_SIGNAL 1

/**
 * Enable APS frame debugging by only looking at the reset or signal
 * frames, and not at the resulting correlated frame.
 * Supported values:
 * 0 - both/CDS (default)
 * 1 - reset read only
 * 2 - signal read only
 */
#define APS_DEBUG_FRAME 0

#define APS_ADC_DEPTH 10

#define APS_ADC_CHANNELS 1

#define APS_ROI_REGIONS_MAX 4

#define IMU6_COUNT 15
#define IMU9_COUNT 21

#define DAVIS_EVENT_TYPES 5
#define DAVIS_SAMPLE_POSITION 4

#define DAVIS_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_FRAME_DEFAULT_SIZE 4
#define DAVIS_IMU_DEFAULT_SIZE 64
#define DAVIS_SAMPLE_DEFAULT_SIZE 512

#define DAVIS_DEVICE_NAME "DAVIS"

#define DAVIS_FX2_DEVICE_PID 0x841B
#define DAVIS_FX2_REQUIRED_LOGIC_REVISION 9880
#define DAVIS_FX2_REQUIRED_FIRMWARE_VERSION 3
#define DAVIS_FX2_USB_CLOCK_FREQ 30

#define DAVIS_FX3_DEVICE_PID 0x841A
#define DAVIS_FX3_REQUIRED_LOGIC_REVISION 9880
#define DAVIS_FX3_REQUIRED_FIRMWARE_VERSION 3
#define DAVIS_FX3_USB_CLOCK_FREQ 80

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

struct davis_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	RingBuffer dataExchangeBuffer;
	atomic_uint_fast32_t dataExchangeBufferSize; // Only takes effect on DataStart() calls!
	atomic_bool dataExchangeBlocking;
	atomic_bool dataExchangeStartProducers;
	atomic_bool dataExchangeStopProducers;
	void (*dataNotifyIncrease)(void *ptr);
	void (*dataNotifyDecrease)(void *ptr);
	void *dataNotifyUserPtr;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	int32_t wrapOverflow;
	int32_t wrapAdd;
	int32_t lastTimestamp;
	int32_t currentTimestamp;
	// DVS specific fields
	uint16_t dvsLastY;
	bool dvsGotY;
	int16_t dvsSizeX;
	int16_t dvsSizeY;
	bool dvsInvertXY;
	// APS specific fields
	int16_t apsSizeX;
	int16_t apsSizeY;
	bool apsInvertXY;
	bool apsFlipX;
	bool apsFlipY;
	bool apsIgnoreEvents;
	bool apsGlobalShutter;
	bool apsResetRead;
	bool apsRGBPixelOffsetDirection; // 0 is increasing, 1 is decreasing.
	int16_t apsRGBPixelOffset;
	uint16_t apsCurrentReadoutType;
	uint16_t apsCountX[APS_READOUT_TYPES_NUM];
	uint16_t apsCountY[APS_READOUT_TYPES_NUM];
	uint16_t *apsCurrentResetFrame;
	uint16_t apsROIUpdate;
	uint16_t apsROITmpData;
	uint16_t apsROISizeX[APS_ROI_REGIONS_MAX];
	uint16_t apsROISizeY[APS_ROI_REGIONS_MAX];
	uint16_t apsROIPositionX[APS_ROI_REGIONS_MAX];
	uint16_t apsROIPositionY[APS_ROI_REGIONS_MAX];
	uint8_t apsExposureFrameUpdate;
	uint32_t apsExposureFrameValue;
	uint32_t apsExposureLastSetValue;
	atomic_bool apsAutoExposureEnabled;
	struct auto_exposure_state apsAutoExposureState;
	// IMU specific fields
	bool imuIgnoreEvents;
	bool imuFlipX;
	bool imuFlipY;
	bool imuFlipZ;
	uint8_t imuCount;
	uint8_t imuTmpData;
	float imuAccelScale;
	float imuGyroScale;
	// Microphone specific fields
	bool micRight;
	uint8_t micCount;
	uint16_t micTmpData;
	// Packet Container state
	caerEventPacketContainer currentPacketContainer;
	atomic_uint_fast32_t maxPacketContainerPacketSize;
	atomic_uint_fast32_t maxPacketContainerInterval;
	int64_t currentPacketContainerCommitTimestamp;
	// Polarity Packet state
	caerPolarityEventPacket currentPolarityPacket;
	int32_t currentPolarityPacketPosition;
	// Frame Packet state
	caerFrameEventPacket currentFramePacket;
	int32_t currentFramePacketPosition;
	// IMU6 Packet state
	caerIMU6EventPacket currentIMU6Packet;
	int32_t currentIMU6PacketPosition;
	// Special Packet state
	caerSpecialEventPacket currentSpecialPacket;
	int32_t currentSpecialPacketPosition;
	// Microphone Sample Packet state
	caerSampleEventPacket currentSamplePacket;
	int32_t currentSamplePacketPosition;
	// Current composite events, for later copy, to not loose them on commits.
	caerFrameEvent currentFrameEvent[APS_ROI_REGIONS_MAX];
	struct caer_imu6_event currentIMU6Event;
	// Debug transfer support (FX3 only).
	bool isFX3Device;
	struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
	atomic_uint_fast32_t activeDebugTransfers;
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

caerDeviceHandle davisCommonOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
caerDeviceHandle davisFX2Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
caerDeviceHandle davisFX3Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);

caerDeviceHandle davisCommonOpenInternal(uint16_t deviceType, uint16_t deviceID, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict);
bool davisCommonClose(caerDeviceHandle cdh);

bool davisCommonSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool davisCommonConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool davisCommonConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool davisCommonDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool davisCommonDataStop(caerDeviceHandle handle);
caerEventPacketContainer davisCommonDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DAVIS_COMMON_H_ */
