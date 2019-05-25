#ifndef LIBCAER_SRC_USB_UTILS_H_
#define LIBCAER_SRC_USB_UTILS_H_

#include "libcaer.h"

#include "devices/usb.h"

#include <libusb.h>
#include <stdatomic.h>

#if defined(HAVE_PTHREADS)
#	include "c11threads_posix.h"
#endif

#define MAX_SERIAL_NUMBER_LENGTH 8

#define USB_DEFAULT_DEVICE_VID 0x152A

#define USB_DEFAULT_DATA_ENDPOINT 0x82

#define VENDOR_REQUEST_FPGA_CONFIG 0xBF
#define VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE 0xC2

enum { TRANS_STOPPED = 0, TRANS_RUNNING = 1 };

struct usb_state {
	// Per-device log-level (USB functions)
	atomic_uint_fast8_t usbLogLevel;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// USB thread state
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	thrd_t usbThread;
	atomic_bool usbThreadRun;
	// USB Data Transfers
	atomic_uint_fast32_t usbBufferNumber;
	atomic_uint_fast32_t usbBufferSize;
	uint8_t dataEndPoint;
	atomic_uint_fast32_t dataTransfersRun;
	mtx_t dataTransfersLock;
	struct libusb_transfer **dataTransfers; // LOCK PROTECTED.
	uint32_t dataTransfersLength;           // LOCK PROTECTED.
	atomic_uint_fast32_t activeDataTransfers;
	uint32_t failedDataTransfers;
	// USB Data Transfers handling callback
	void (*usbDataCallback)(void *usbDataCallbackPtr, const uint8_t *buffer, size_t bytesSent);
	void *usbDataCallbackPtr;
	// USB Data Transfers shutdown callback
	void (*usbShutdownCallback)(void *usbShutdownCallbackPtr);
	void *usbShutdownCallbackPtr;
};

typedef struct usb_state *usbState;

struct usb_info {
	uint8_t busNumber;
	uint8_t devAddress;
	char serialNumber[MAX_SERIAL_NUMBER_LENGTH + 1];
	bool errorOpen;
	bool errorVersion;
	int16_t firmwareVersion;
	int16_t logicVersion;
};

ssize_t usbDeviceFind(uint16_t devVID, uint16_t devPID, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion,
	struct usb_info **foundUSBDevices);

bool usbDeviceOpen(usbState state, uint16_t devVID, uint16_t devPID, uint8_t busNumber, uint8_t devAddress,
	const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion,
	struct usb_info *deviceUSBInfo);
void usbDeviceClose(usbState state);

void usbSetThreadName(usbState state, const char *threadName);
void usbSetDataCallback(usbState state,
	void (*usbDataCallback)(void *usbDataCallbackPtr, const uint8_t *buffer, size_t bytesSent),
	void *usbDataCallbackPtr);
void usbSetShutdownCallback(
	usbState state, void (*usbShutdownCallback)(void *usbShutdownCallbackPtr), void *usbShutdownCallbackPtr);
void usbSetDataEndpoint(usbState state, uint8_t dataEndPoint);
void usbSetTransfersNumber(usbState state, uint32_t transfersNumber);
void usbSetTransfersSize(usbState state, uint32_t transfersSize);
uint32_t usbGetTransfersNumber(usbState state);
uint32_t usbGetTransfersSize(usbState state);

static inline bool usbConfigSet(usbState state, uint8_t paramAddr, uint32_t param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
			usbSetTransfersNumber(state, param);
			break;

		case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
			usbSetTransfersSize(state, param);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

static inline bool usbConfigGet(usbState state, uint8_t paramAddr, uint32_t *param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
			*param = usbGetTransfersNumber(state);
			break;

		case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
			*param = usbGetTransfersSize(state);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

char *usbGenerateDeviceString(struct usb_info usbInfo, const char *deviceName, uint16_t deviceID);

bool usbThreadStart(usbState state);
void usbThreadStop(usbState state);

static inline bool usbDataTransfersAreRunning(usbState state) {
	return (atomic_load(&state->dataTransfersRun) == TRANS_RUNNING);
}
bool usbDataTransfersStart(usbState state);
void usbDataTransfersStop(usbState state);

bool usbControlTransferOutAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status), void *controlOutCallbackPtr);
bool usbControlTransferInAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, size_t dataSize,
	void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize),
	void *controlInCallbackPtr);
bool usbControlTransferOut(
	usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, size_t dataSize);
bool usbControlTransferIn(
	usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, size_t dataSize);

// SPI config via USB implementation.
#include "spi_config_interface.h"

struct usb_config_receive_struct {
	void (*configReceiveCallback)(void *configReceiveCallbackPtr, int status, uint32_t param);
	void *configReceiveCallbackPtr;
};

typedef struct usb_config_receive_struct *usbConfigReceive;

static void spiConfigReceiveCallback(
	void *configReceiveCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize);

static bool spiConfigSendMultiple(void *state, spiConfigParams configs, uint16_t numConfigs) {
	for (size_t i = 0; i < numConfigs; i++) {
		// Param must be in big-endian format.
		configs[i].param = htobe32(configs[i].param);
	}

	return (usbControlTransferOut(state, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, numConfigs, 0, (uint8_t *) configs,
		sizeof(struct spi_config_params) * numConfigs));
}

static bool spiConfigSendMultipleAsync(void *state, spiConfigParams configs, uint16_t numConfigs,
	void (*configSendCallback)(void *configSendCallbackPtr, int status), void *configSendCallbackPtr) {
	for (size_t i = 0; i < numConfigs; i++) {
		// Param must be in big-endian format.
		configs[i].param = htobe32(configs[i].param);
	}

	return (usbControlTransferOutAsync(state, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, numConfigs, 0, (uint8_t *) configs,
		sizeof(struct spi_config_params) * numConfigs, configSendCallback, configSendCallbackPtr));
}

static bool spiConfigSend(void *state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	uint8_t spiConfig[4] = {0};

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	return (
		usbControlTransferOut(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig)));
}

static bool spiConfigSendAsync(void *state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param,
	void (*configSendCallback)(void *configSendCallbackPtr, int status), void *configSendCallbackPtr) {
	uint8_t spiConfig[4] = {0};

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	return (usbControlTransferOutAsync(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig,
		sizeof(spiConfig), configSendCallback, configSendCallbackPtr));
}

static bool spiConfigReceive(void *state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param) {
	uint8_t spiConfig[4] = {0};

	if (!usbControlTransferIn(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig))) {
		return (false);
	}

	*param = 0;
	*param |= U32T(spiConfig[0] << 24);
	*param |= U32T(spiConfig[1] << 16);
	*param |= U32T(spiConfig[2] << 8);
	*param |= U32T(spiConfig[3] << 0);

	return (true);
}

static bool spiConfigReceiveAsync(void *state, uint8_t moduleAddr, uint8_t paramAddr,
	void (*configReceiveCallback)(void *configReceiveCallbackPtr, int status, uint32_t param),
	void *configReceiveCallbackPtr) {
	usbConfigReceive config = calloc(1, sizeof(*config));
	if (config == NULL) {
		return (false);
	}

	config->configReceiveCallback    = configReceiveCallback;
	config->configReceiveCallbackPtr = configReceiveCallbackPtr;

	bool retVal = usbControlTransferInAsync(
		state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, sizeof(uint32_t), &spiConfigReceiveCallback, config);
	if (!retVal) {
		free(config);

		return (false);
	}

	return (true);
}

static void spiConfigReceiveCallback(
	void *configReceiveCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize) {
	usbConfigReceive config = configReceiveCallbackPtr;

	uint32_t param = 0;

	if ((status == LIBUSB_TRANSFER_COMPLETED) && (bufferSize == sizeof(uint32_t))) {
		param |= U32T(buffer[0] << 24);
		param |= U32T(buffer[1] << 16);
		param |= U32T(buffer[2] << 8);
		param |= U32T(buffer[3] << 0);
	}

	if (config->configReceiveCallback != NULL) {
		(*config->configReceiveCallback)(config->configReceiveCallbackPtr, status, param);
	}

	free(config);
}

#endif /* LIBCAER_SRC_USB_UTILS_H_ */
