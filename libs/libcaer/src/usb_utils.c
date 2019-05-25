#include "usb_utils.h"

struct usb_control_struct {
	union {
		void (*controlOutCallback)(void *controlOutCallbackPtr, int status);
		void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize);
	};
	void *controlCallbackPtr;
};

typedef struct usb_control_struct *usbControl;

struct usb_data_completion_struct {
	atomic_uint_fast32_t completed;
	uint8_t *data;
	size_t dataSize;
};

typedef struct usb_data_completion_struct *usbDataCompletion;

static void caerUSBLog(enum caer_log_level logLevel, usbState state, const char *format, ...) ATTRIBUTE_FORMAT(3);
static int usbThreadRun(void *usbStatePtr);
static bool usbAllocateTransfers(usbState state);
static void usbCancelAndDeallocateTransfers(usbState state);
static void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer);
static bool usbControlTransferAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status),
	void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize),
	void *controlCallbackPtr, bool directionOut);
static void LIBUSB_CALL usbControlOutCallback(struct libusb_transfer *transfer);
static void LIBUSB_CALL usbControlInCallback(struct libusb_transfer *transfer);
static void syncControlOutCallback(void *controlOutCallbackPtr, int status);
static void syncControlInCallback(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize);

static inline bool checkActiveConfigAndClaim(libusb_device_handle *devHandle) {
	// Check that the active configuration is set to number 1. If not, do so.
	int activeConfiguration;
	if (libusb_get_configuration(devHandle, &activeConfiguration) != LIBUSB_SUCCESS) {
		return (false);
	}

	if (activeConfiguration != 1) {
		if (libusb_set_configuration(devHandle, 1) != LIBUSB_SUCCESS) {
			return (false);
		}
	}

	// Claim interface 0 (default).
	if (libusb_claim_interface(devHandle, 0) != LIBUSB_SUCCESS) {
		return (false);
	}

	return (true);
}

static void caerUSBLog(enum caer_log_level logLevel, usbState state, const char *format, ...) {
	// Only log messages above the specified severity level.
	uint8_t systemLogLevel = atomic_load_explicit(&state->usbLogLevel, memory_order_relaxed);

	if (logLevel > systemLogLevel) {
		return;
	}

	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(systemLogLevel, logLevel, state->usbThreadName, format, argumentList);
	va_end(argumentList);
}

ssize_t usbDeviceFind(uint16_t devVID, uint16_t devPID, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion,
	struct usb_info **foundUSBDevices) {
	// Set to NULL initially (for error return).
	*foundUSBDevices = NULL;

	// libusb may create its own threads at this stage, so we temporarily set
	// a different thread name.
	char originalThreadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	thrd_get_name(originalThreadName, MAX_THREAD_NAME_LENGTH);
	originalThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	thrd_set_name("USBDiscovery");

	int res = libusb_init(NULL);

	thrd_set_name(originalThreadName);

	if (res != LIBUSB_SUCCESS) {
		return (-1);
	}

	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(NULL, &devicesList);

	if (result < 0) {
		libusb_exit(NULL);

		return (-1);
	}

	// Cycle thorough all discovered devices and count matches.
	size_t matches = 0;

	for (size_t i = 0; i < (size_t) result; i++) {
		struct libusb_device_descriptor devDesc;

		if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
			continue;
		}

		// Check if this is the device we want (VID/PID).
		if ((devDesc.idVendor == devVID) && (devDesc.idProduct == devPID)) {
			matches++;
		}
	}

	// No matches?
	if (matches == 0) {
		libusb_free_device_list(devicesList, true);
		libusb_exit(NULL);

		return (0);
	}

	// Now that we know how many there are, we can allocate the proper
	// amount of memory to hold the result.
	*foundUSBDevices = calloc(matches, sizeof(struct usb_info));
	if (*foundUSBDevices == NULL) {
		libusb_free_device_list(devicesList, true);
		libusb_exit(NULL);

		return (-1);
	}

	matches = 0; // Use as counter again.

	for (size_t i = 0; i < (size_t) result; i++) {
		struct libusb_device_descriptor devDesc;

		if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
			continue;
		}

		// Check if this is the device we want (VID/PID).
		if ((devDesc.idVendor == devVID) && (devDesc.idProduct == devPID)) {
			// Get USB bus number and device address from descriptors.
			(*foundUSBDevices)[matches].busNumber  = libusb_get_bus_number(devicesList[i]);
			(*foundUSBDevices)[matches].devAddress = libusb_get_device_address(devicesList[i]);

			libusb_device_handle *devHandle = NULL;

			if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
				(*foundUSBDevices)[matches].errorOpen = true;
				matches++;
				continue;
			}

			// Get serial number.
			char serialNumber[MAX_SERIAL_NUMBER_LENGTH + 1] = {0};
			int getStringDescResult                         = libusb_get_string_descriptor_ascii(
                devHandle, 3, (unsigned char *) serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

			// Check serial number success and length.
			if ((getStringDescResult < 0) || (getStringDescResult > MAX_SERIAL_NUMBER_LENGTH)) {
				libusb_close(devHandle);

				(*foundUSBDevices)[matches].errorOpen = true;
				matches++;
				continue;
			}

			strncpy((*foundUSBDevices)[matches].serialNumber, serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

			// Check that the active configuration is set to number 1. If not, do so.
			// Then claim interface 0 (default).
			if (!checkActiveConfigAndClaim(devHandle)) {
				libusb_close(devHandle);

				(*foundUSBDevices)[matches].errorOpen = true;
				matches++;
				continue;
			}

			// Verify device firmware version.
			bool firmwareVersionOK = true;

			if (requiredFirmwareVersion >= 0) {
				if (U8T(devDesc.bcdDevice & 0x00FF) != U8T(requiredFirmwareVersion)) {
					firmwareVersionOK = false;
				}

				(*foundUSBDevices)[matches].firmwareVersion = I16T(U8T(devDesc.bcdDevice & 0x00FF));
			}

			// Verify device logic version.
			bool logicVersionOK = true;

			if (requiredLogicRevision >= 0) {
				// Communication with device open, get logic version information.
				uint32_t param32 = 0;

				// Get logic version from generic SYSINFO module.
				uint8_t spiConfig[4] = {0};

				if (libusb_control_transfer(devHandle,
						LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
						VENDOR_REQUEST_FPGA_CONFIG, 6, 0, spiConfig, sizeof(spiConfig), 0)
					!= sizeof(spiConfig)) {
					libusb_release_interface(devHandle, 0);
					libusb_close(devHandle);

					(*foundUSBDevices)[matches].errorVersion = true;
					matches++;
					continue;
				}

				param32 |= U32T(spiConfig[0] << 24);
				param32 |= U32T(spiConfig[1] << 16);
				param32 |= U32T(spiConfig[2] << 8);
				param32 |= U32T(spiConfig[3] << 0);

				// Verify device logic version.
				if (param32 != U32T(requiredLogicRevision)) {
					logicVersionOK = false;
				}

				(*foundUSBDevices)[matches].logicVersion = I16T(param32);
			}

			// If any of the version checks failed, stop.
			if (!firmwareVersionOK || !logicVersionOK) {
				(*foundUSBDevices)[matches].errorVersion = true;
			}

			libusb_release_interface(devHandle, 0);
			libusb_close(devHandle);

			matches++;
		}
	}

	libusb_free_device_list(devicesList, true);
	libusb_exit(NULL);

	return ((ssize_t) matches);
}

bool usbDeviceOpen(usbState state, uint16_t devVID, uint16_t devPID, uint8_t busNumber, uint8_t devAddress,
	const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion,
	struct usb_info *deviceUSBInfo) {
	errno = 0;

	// Ensure no content.
	memset(deviceUSBInfo, 0, sizeof(struct usb_info));

	// Search for device and open it.
	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	// libusb may create its own threads at this stage, so we temporarily set
	// a different thread name.
	char originalThreadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	thrd_get_name(originalThreadName, MAX_THREAD_NAME_LENGTH);
	originalThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	thrd_set_name(state->usbThreadName);

	int res = libusb_init(&state->deviceContext);

	thrd_set_name(originalThreadName);

	if (res != LIBUSB_SUCCESS) {
		caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to initialize libusb context. Error: %d.", res);
		errno = CAER_ERROR_RESOURCE_ALLOCATION;
		return (false);
	}

	bool openingSpecificUSBAddr = ((busNumber > 0) && (devAddress > 0));
	bool openingSpecificSerial  = ((serialNumber != NULL) && !caerStrEquals(serialNumber, ""));

	libusb_device_handle *devHandle = NULL;
	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(state->deviceContext, &devicesList);

	if (result >= 0) {
		// Cycle thorough all discovered devices and find a match.
		for (size_t i = 0; i < (size_t) result; i++) {
			struct libusb_device_descriptor devDesc;

			if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
				continue;
			}

			// Check if this is the device we want (VID/PID).
			if ((devDesc.idVendor == devVID) && (devDesc.idProduct == devPID)) {
				// If a USB port restriction is given, honor it first.
				uint8_t devBusNumber = libusb_get_bus_number(devicesList[i]);
				if ((busNumber > 0) && (devBusNumber != busNumber)) {
					caerUSBLog(CAER_LOG_DEBUG, state,
						"USB bus number restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8 ").",
						busNumber, devBusNumber);

					continue;
				}
				deviceUSBInfo->busNumber = devBusNumber;

				uint8_t devDevAddress = libusb_get_device_address(devicesList[i]);
				if ((devAddress > 0) && (devDevAddress != devAddress)) {
					caerUSBLog(CAER_LOG_DEBUG, state,
						"USB device address restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8
						").",
						devAddress, devDevAddress);

					continue;
				}
				deviceUSBInfo->devAddress = devDevAddress;

				if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
					devHandle = NULL;

					caerUSBLog((openingSpecificUSBAddr) ? (CAER_LOG_ERROR) : (CAER_LOG_INFO), state,
						"Failed to open USB device. This usually happens due to permission or driver issues, or "
						"because the device is already in use.");

					continue;
				}

				// Get the device's serial number.
				char deviceSerialNumber[MAX_SERIAL_NUMBER_LENGTH + 1] = {0};
				int getStringDescResult = libusb_get_string_descriptor_ascii(devHandle, devDesc.iSerialNumber,
					(unsigned char *) deviceSerialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

				// Check serial number success and length.
				if ((getStringDescResult < 0) || (getStringDescResult > MAX_SERIAL_NUMBER_LENGTH)) {
					libusb_close(devHandle);
					devHandle = NULL;

					caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to get a valid USB serial number.");

					continue;
				}

				// Check the serial number restriction, if any is present.
				if (openingSpecificSerial && !caerStrEquals(serialNumber, deviceSerialNumber)) {
					libusb_close(devHandle);
					devHandle = NULL;

					caerUSBLog(CAER_LOG_INFO, state,
						"USB serial number restriction is present (%s), this device didn't match it (%s).",
						serialNumber, deviceSerialNumber);

					continue;
				}

				// Copy serial number over.
				strncpy((char *) &deviceUSBInfo->serialNumber, deviceSerialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

				// Check that the active configuration is set to number 1. If not, do so.
				// Then claim interface 0 (default).
				if (!checkActiveConfigAndClaim(devHandle)) {
					libusb_close(devHandle);
					devHandle = NULL;

					caerUSBLog((openingSpecificUSBAddr || openingSpecificSerial) ? (CAER_LOG_ERROR) : (CAER_LOG_INFO),
						state,
						"Failed to claim USB interface. This usually happens because the device is already in use.");

					continue;
				}

				// Verify device firmware version.
				bool firmwareVersionOK = true;

				if (requiredFirmwareVersion >= 0) {
					if (U8T(devDesc.bcdDevice & 0x00FF) != U8T(requiredFirmwareVersion)) {
						caerUSBLog(CAER_LOG_ERROR, state,
							"Device firmware version incorrect. You have version %" PRIu8 "; but version %" PRIu8
							" is required. Please update by following the Flashy documentation at "
							"'https://inivation.com/support/software/reflashing/'.",
							U8T(devDesc.bcdDevice & 0x00FF), U8T(requiredFirmwareVersion));

						firmwareVersionOK = false;
						errno             = CAER_ERROR_FW_VERSION;
					}

					deviceUSBInfo->firmwareVersion = I16T(U8T(devDesc.bcdDevice & 0x00FF));
				}

				// Verify device logic version.
				bool logicVersionOK = true;

				if (requiredLogicRevision >= 0) {
					// Communication with device open, get logic version information.
					uint32_t param32 = 0;

					// Get logic version from generic SYSINFO module.
					uint8_t spiConfig[4] = {0};

					if (libusb_control_transfer(devHandle,
							LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
							VENDOR_REQUEST_FPGA_CONFIG, 6, 0, spiConfig, sizeof(spiConfig), 0)
						!= sizeof(spiConfig)) {
						libusb_release_interface(devHandle, 0);
						libusb_close(devHandle);
						devHandle = NULL;

						caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to get current logic version.");

						errno = CAER_ERROR_COMMUNICATION;
						continue;
					}

					param32 |= U32T(spiConfig[0] << 24);
					param32 |= U32T(spiConfig[1] << 16);
					param32 |= U32T(spiConfig[2] << 8);
					param32 |= U32T(spiConfig[3] << 0);

					// Verify device logic version.
					if (param32 != U32T(requiredLogicRevision)) {
						caerUSBLog(CAER_LOG_ERROR, state,
							"Device logic version incorrect. You have version %" PRIu32 "; but version %" PRIu32
							" is required. Please update by following the Flashy documentation at "
							"'https://inivation.com/support/software/reflashing/'.",
							param32, U32T(requiredLogicRevision));

						logicVersionOK = false;
						errno          = CAER_ERROR_LOGIC_VERSION;
					}

					deviceUSBInfo->logicVersion = I16T(param32);
				}

				// If any of the version checks failed, stop.
				if (!firmwareVersionOK || !logicVersionOK) {
					libusb_release_interface(devHandle, 0);
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				// Initialize transfers mutex.
				if (mtx_init(&state->dataTransfersLock, mtx_plain) != thrd_success) {
					libusb_release_interface(devHandle, 0);
					libusb_close(devHandle);
					devHandle = NULL;

					caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to initialize USB transfer mutex.");

					errno = CAER_ERROR_RESOURCE_ALLOCATION;
					continue;
				}

				break;
			}
		}

		libusb_free_device_list(devicesList, true);
	}

	// Found and configured it!
	if (devHandle != NULL) {
		state->deviceHandle = devHandle;
		errno               = 0; // Ensure reset on success.
		return (true);
	}

	// Didn't find anything.
	libusb_exit(state->deviceContext);
	state->deviceContext = NULL;

	// Filter errno due to libusb setting it to other values even if everything
	// above returns no errors (like EAGAIN(11)). All errnos we want to set
	// and track are negative, so any positive errnos are wrong.
	if (errno > 0) {
		errno = 0;
	}

	// If not set previously to a more precise error,
	// set here to the generic open error.
	if (errno == 0) {
		errno = CAER_ERROR_OPEN_ACCESS;
	}

	return (false);
}

void usbDeviceClose(usbState state) {
	mtx_destroy(&state->dataTransfersLock);

	// Release interface 0 (default).
	libusb_release_interface(state->deviceHandle, 0);

	libusb_close(state->deviceHandle);

	libusb_exit(state->deviceContext);
}

void usbSetThreadName(usbState state, const char *threadName) {
	strncpy(state->usbThreadName, threadName, MAX_THREAD_NAME_LENGTH);
	state->usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';
}

void usbSetDataCallback(usbState state,
	void (*usbDataCallback)(void *usbDataCallbackPtr, const uint8_t *buffer, size_t bytesSent),
	void *usbDataCallbackPtr) {
	state->usbDataCallback    = usbDataCallback;
	state->usbDataCallbackPtr = usbDataCallbackPtr;
}

void usbSetShutdownCallback(
	usbState state, void (*usbShutdownCallback)(void *usbShutdownCallbackPtr), void *usbShutdownCallbackPtr) {
	state->usbShutdownCallback    = usbShutdownCallback;
	state->usbShutdownCallbackPtr = usbShutdownCallbackPtr;

	atomic_thread_fence(memory_order_seq_cst);
}

void usbSetDataEndpoint(usbState state, uint8_t dataEndPoint) {
	state->dataEndPoint = dataEndPoint;
}

void usbSetTransfersNumber(usbState state, uint32_t transfersNumber) {
	atomic_store(&state->usbBufferNumber, transfersNumber);

	// Cancel transfers, wait for them to terminate, deallocate, and
	// then reallocate with new size/number.
	mtx_lock(&state->dataTransfersLock);
	if (usbDataTransfersAreRunning(state)) {
		usbCancelAndDeallocateTransfers(state);

		// Check again, for exceptional shutdown may have set this to false.
		if (usbDataTransfersAreRunning(state)) {
			usbAllocateTransfers(state);
		}
	}
	mtx_unlock(&state->dataTransfersLock);
}

void usbSetTransfersSize(usbState state, uint32_t transfersSize) {
	atomic_store(&state->usbBufferSize, transfersSize);

	// Cancel transfers, wait for them to terminate, deallocate, and
	// then reallocate with new size/number.
	mtx_lock(&state->dataTransfersLock);
	if (usbDataTransfersAreRunning(state)) {
		usbCancelAndDeallocateTransfers(state);

		// Check again, for exceptional shutdown may have set this to false.
		if (usbDataTransfersAreRunning(state)) {
			usbAllocateTransfers(state);
		}
	}
	mtx_unlock(&state->dataTransfersLock);
}

uint32_t usbGetTransfersNumber(usbState state) {
	return (U32T(atomic_load(&state->usbBufferNumber)));
}

uint32_t usbGetTransfersSize(usbState state) {
	return (U32T(atomic_load(&state->usbBufferSize)));
}

char *usbGenerateDeviceString(struct usb_info usbInfo, const char *deviceName, uint16_t deviceID) {
	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]",
		deviceName, deviceID, usbInfo.serialNumber, usbInfo.busNumber, usbInfo.devAddress);

	char *fullLogString = malloc(fullLogStringLength + 1);
	if (fullLogString == NULL) {
		return (NULL);
	}

	snprintf(fullLogString, fullLogStringLength + 1, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]", deviceName,
		deviceID, usbInfo.serialNumber, usbInfo.busNumber, usbInfo.devAddress);

	return (fullLogString);
}

bool usbThreadStart(usbState state) {
	// Start USB thread.
	if ((errno = thrd_create(&state->usbThread, &usbThreadRun, state)) != thrd_success) {
		caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to create USB thread. Error: %d.", errno);
		return (false);
	}

	// Wait for USB thread to be ready.
	while (!atomic_load_explicit(&state->usbThreadRun, memory_order_relaxed)) {
		;
	}

	return (true);
}

void usbThreadStop(usbState state) {
	// Shut down USB thread.
	atomic_store(&state->usbThreadRun, false);

	// Wait for USB thread to terminate.
	if ((errno = thrd_join(state->usbThread, NULL)) != thrd_success) {
		// This should never happen!
		caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to join USB thread. Error: %d.", errno);
	}
}

// This thread handles all USB events exclusively, from when it starts up
// after device open to when it shuts down at device close.
static int usbThreadRun(void *usbStatePtr) {
	usbState state = usbStatePtr;

	caerUSBLog(CAER_LOG_DEBUG, state, "Starting USB thread ...");

	// Set thread name.
	thrd_set_name(state->usbThreadName);

	// Signal data thread ready back to start function.
	atomic_store(&state->usbThreadRun, true);

	caerUSBLog(CAER_LOG_DEBUG, state, "USB thread running.");

	// Handle USB events (10 millisecond timeout).
	struct timeval te = {.tv_sec = 0, .tv_usec = 10000};

	while (atomic_load_explicit(&state->usbThreadRun, memory_order_relaxed)) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	caerUSBLog(CAER_LOG_DEBUG, state, "USB thread shut down.");

	return (EXIT_SUCCESS);
}

bool usbDataTransfersStart(usbState state) {
	mtx_lock(&state->dataTransfersLock);
	bool retVal = usbAllocateTransfers(state);
	if (retVal) {
		atomic_store(&state->dataTransfersRun, TRANS_RUNNING);
	}
	mtx_unlock(&state->dataTransfersLock);

	return (retVal);
}

void usbDataTransfersStop(usbState state) {
	mtx_lock(&state->dataTransfersLock);
	atomic_store(&state->dataTransfersRun, TRANS_STOPPED);
	usbCancelAndDeallocateTransfers(state);
	mtx_unlock(&state->dataTransfersLock);
}

// MUST LOCK ON 'dataTransfersLock'.
static bool usbAllocateTransfers(usbState state) {
	uint32_t bufferNum  = usbGetTransfersNumber(state);
	uint32_t bufferSize = usbGetTransfersSize(state);

	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerUSBLog(CAER_LOG_CRITICAL, state, "Failed to allocate memory for %" PRIu32 " libusb transfers. Error: %d.",
			bufferNum, errno);
		return (false);
	}
	state->dataTransfersLength = bufferNum;

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerUSBLog(CAER_LOG_CRITICAL, state, "Unable to allocate further libusb transfers (%zu of %" PRIu32 ").", i,
				bufferNum);
			continue;
		}

		// Create data buffer.
		state->dataTransfers[i]->length = (int) bufferSize;
		state->dataTransfers[i]->buffer = malloc(bufferSize);
		if (state->dataTransfers[i]->buffer == NULL) {
			caerUSBLog(
				CAER_LOG_CRITICAL, state, "Unable to allocate buffer for libusb transfer %zu. Error: %d.", i, errno);

			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		state->dataTransfers[i]->dev_handle = state->deviceHandle;
		state->dataTransfers[i]->endpoint   = state->dataEndPoint;
		state->dataTransfers[i]->type       = LIBUSB_TRANSFER_TYPE_BULK;
		state->dataTransfers[i]->callback   = &usbDataTransferCallback;
		state->dataTransfers[i]->user_data  = state;
		state->dataTransfers[i]->timeout    = 0;
		state->dataTransfers[i]->flags      = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->dataTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_fetch_add(&state->activeDataTransfers, 1);
		}
		else {
			caerUSBLog(CAER_LOG_CRITICAL, state, "Unable to submit libusb transfer %zu. Error: %s (%d).", i,
				libusb_strerror(errno), errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;
		}
	}

	if (atomic_load(&state->activeDataTransfers) == 0) {
		// Didn't manage to allocate any USB transfers, free array memory and log failure.
		free(state->dataTransfers);
		state->dataTransfers       = NULL;
		state->dataTransfersLength = 0;

		caerUSBLog(CAER_LOG_CRITICAL, state, "Unable to allocate any libusb transfers.");
		return (false);
	}

	return (true);
}

// MUST LOCK ON 'dataTransfersLock'.
static void usbCancelAndDeallocateTransfers(usbState state) {
	// Wait for all transfers to go away.
	struct timespec waitForTerminationSleep = {.tv_sec = 0, .tv_nsec = 1000000};

	while (atomic_load(&state->activeDataTransfers) > 0) {
		// Continue trying to cancel all transfers until there are none left.
		// It seems like one cancel pass is not enough and some hang around.
		for (size_t i = 0; i < state->dataTransfersLength; i++) {
			if (state->dataTransfers[i] != NULL) {
				errno = libusb_cancel_transfer(state->dataTransfers[i]);
				if ((errno != LIBUSB_SUCCESS) && (errno != LIBUSB_ERROR_NOT_FOUND)) {
					caerUSBLog(CAER_LOG_CRITICAL, state, "Unable to cancel libusb transfer %zu. Error: %s (%d).", i,
						libusb_strerror(errno), errno);
					// Proceed with trying to cancel all transfers regardless of errors.
				}
			}
		}

		// Sleep for 1ms to avoid busy loop.
		thrd_sleep(&waitForTerminationSleep, NULL);
	}

	// No more transfers in flight, deallocate them all here.
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		if (state->dataTransfers[i] != NULL) {
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;
		}
	}

	// And lastly free the transfers array.
	free(state->dataTransfers);
	state->dataTransfers       = NULL;
	state->dataTransfersLength = 0;
}

static void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer) {
	usbState state = transfer->user_data;

	// Completed or cancelled transfers are what we expect to handle here, so
	// if they do have data attached, try to parse them.
	if (((transfer->status == LIBUSB_TRANSFER_COMPLETED) || (transfer->status == LIBUSB_TRANSFER_CANCELLED))
		&& (transfer->actual_length > 0)) {
		// Handle data.
		(*state->usbDataCallback)(state->usbDataCallbackPtr, transfer->buffer, (size_t) transfer->actual_length);
	}

	// Only status that indicates a new transfer can be really submitted is
	// COMPLETED. TIMED_OUT is impossible, and ERROR/STALL/NO_DEVICE/CANCELLED
	// are not recoverable, as all of them appear on different OSes when a
	// device is physically unplugged for example.
	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counters and exiting.
	// Freeing the transfers is taken care of by usbCancelAndDeallocateTransfers().
	// 'activeDataTransfers' drops to zero in three cases:
	// - the device went away
	// - the data transfers were stopped, which cancels all transfers
	// - the USB buffer number/size changed, which cancels all transfers
	// The second and third case are intentional user actions, so we don't notify.
	// In the first case, the last transfer to go away calls the shutdown
	// callback and notifies that we're exiting, and not via normal cancellation.
	// On MacOS X, when an error occurs (like device being unplugged), not all
	// transfers return with the expected error code, but only one, the rest returns
	// LIBUSB_TRANSFER_CANCELLED, which we only expect on actual user cancel actions.
	// To track this, we count the transfers that did fail with any other error code,
	// and use that information to determine if the shutdown was intentional by the
	// user or not. If not, at least one transfer would fail with a non-cancel error.
	if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		// This also captures COMPLETED but with re-submit failure.
		state->failedDataTransfers++;
	}

	// Transfers are handled sequentially always in the same thread, so these
	// reads here are correct.
	if ((atomic_load(&state->activeDataTransfers) == 1) && (state->failedDataTransfers > 0)) {
		// Ensure run is set to false on exceptional shut-down.
		atomic_store(&state->dataTransfersRun, TRANS_STOPPED);

		// We make sure to first set 'dataTransfersRun' to false on exceptional
		// shut-down, before doing the subtraction, so that anyone waiting on
		// 'activeDataTransfers' to become zero, will see RUN changed to false too.
		atomic_store(&state->activeDataTransfers, 0);

		// Call exceptional shut-down callback,
		if (state->usbShutdownCallback != NULL) {
			state->usbShutdownCallback(state->usbShutdownCallbackPtr);
		}
	}
	else {
		// Normal shutdown/count-down.
		atomic_fetch_sub(&state->activeDataTransfers, 1);
	}

	// Clear error tracking counter on last exit.
	if ((atomic_load(&state->activeDataTransfers) == 0) && (state->failedDataTransfers > 0)) {
		state->failedDataTransfers = 0;
	}
}

static bool usbControlTransferAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status),
	void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize),
	void *controlCallbackPtr, bool directionOut) {
	// If doing IN, data must always be NULL, the callback will handle it.
	if ((!directionOut) && (data != NULL)) {
		return (false);
	}

	// If doing OUT and data is NULL (no data), dataSize must be zero!
	if (directionOut && (data == NULL) && (dataSize != 0)) {
		return (false);
	}

	struct libusb_transfer *controlTransfer = libusb_alloc_transfer(0);
	if (controlTransfer == NULL) {
		return (false);
	}

	// Create data buffer.
	uint8_t *controlTransferBuffer
		= calloc(1, (LIBUSB_CONTROL_SETUP_SIZE + dataSize + sizeof(struct usb_control_struct)));
	if (controlTransferBuffer == NULL) {
		caerUSBLog(
			CAER_LOG_CRITICAL, state, "Unable to allocate buffer for libusb control transfer. Error: %d.", errno);

		libusb_free_transfer(controlTransfer);

		return (false);
	}

	// Put additional data in the unused part of the transfer buffer, this way
	// all memory is in one block and freed when the transfer is freed.
	usbControl extraControlData = (usbControl) &controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + dataSize];

	if (controlOutCallback != NULL) {
		extraControlData->controlOutCallback = controlOutCallback;
	}
	if (controlInCallback != NULL) {
		extraControlData->controlInCallback = controlInCallback;
	}
	extraControlData->controlCallbackPtr = controlCallbackPtr;

	// Initialize Transfer.
	uint8_t direction = (directionOut) ? (LIBUSB_ENDPOINT_OUT) : (LIBUSB_ENDPOINT_IN);
	libusb_fill_control_setup(controlTransferBuffer, direction | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		bRequest, wValue, wIndex, U16T(dataSize));

	libusb_transfer_cb_fn controlCallback = (directionOut) ? (&usbControlOutCallback) : (&usbControlInCallback);
	libusb_fill_control_transfer(
		controlTransfer, state->deviceHandle, controlTransferBuffer, controlCallback, extraControlData, 0);

	controlTransfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	// Put data in buffer. Only for OUT transfers, 'data' is always NULL for IN transfers.
	if (data != NULL) {
		memcpy(controlTransferBuffer + LIBUSB_CONTROL_SETUP_SIZE, data, dataSize);
	}

	if ((errno = libusb_submit_transfer(controlTransfer)) != LIBUSB_SUCCESS) {
		// The transfer buffer is freed automatically here thanks to
		// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
		libusb_free_transfer(controlTransfer);

		return (false);
	}

	return (true);
}

bool usbControlTransferOutAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status), void *controlOutCallbackPtr) {
	return (usbControlTransferAsync(
		state, bRequest, wValue, wIndex, data, dataSize, controlOutCallback, NULL, controlOutCallbackPtr, true));
}

bool usbControlTransferInAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, size_t dataSize,
	void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize),
	void *controlInCallbackPtr) {
	return (usbControlTransferAsync(
		state, bRequest, wValue, wIndex, NULL, dataSize, NULL, controlInCallback, controlInCallbackPtr, false));
}

// Async USB control exists to avoid the problem described in
// https://sourceforge.net/p/libusb/mailman/message/34129129/
// where the config function after the loop is never entered.
static void LIBUSB_CALL usbControlOutCallback(struct libusb_transfer *transfer) {
	usbControl extraControlData = transfer->user_data;

	if (extraControlData->controlOutCallback != NULL) {
		(*extraControlData->controlOutCallback)(extraControlData->controlCallbackPtr, (int) transfer->status);
	}

	libusb_free_transfer(transfer);
}

static void LIBUSB_CALL usbControlInCallback(struct libusb_transfer *transfer) {
	usbControl extraControlData = transfer->user_data;

	if (extraControlData->controlInCallback != NULL) {
		(*extraControlData->controlInCallback)(extraControlData->controlCallbackPtr, (int) transfer->status,
			libusb_control_transfer_get_data(transfer), (size_t) transfer->actual_length);
	}

	libusb_free_transfer(transfer);
}

// Implement synchronous API over asynchronous one: wait till callbacks are done.
// This is done again here because the libusb API is unsuitable as it does its
// own event handling, which we want to fully delegate to the USB thread.
bool usbControlTransferOut(
	usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, size_t dataSize) {
	atomic_uint_fast32_t completed = ATOMIC_VAR_INIT(0);

	bool retVal = usbControlTransferOutAsync(
		state, bRequest, wValue, wIndex, data, dataSize, &syncControlOutCallback, &completed);
	if (!retVal) {
		// Failed to send out async request.
		return (false);
	}

	// Request is out and will be handled at some point by USB thread, wait on that.
	struct timespec waitForCompletionSleep = {.tv_sec = 0, .tv_nsec = 100000};

	while (!atomic_load(&completed)) {
		// Sleep for 100µs to avoid busy loop.
		thrd_sleep(&waitForCompletionSleep, NULL);
	}

	if (atomic_load(&completed) == 1) {
		// Success.
		return (true);
	}
	else {
		// Failure.
		return (false);
	}
}

bool usbControlTransferIn(
	usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, size_t dataSize) {
	struct usb_data_completion_struct dataCompletion = {ATOMIC_VAR_INIT(0), data, dataSize};

	bool retVal
		= usbControlTransferInAsync(state, bRequest, wValue, wIndex, dataSize, &syncControlInCallback, &dataCompletion);
	if (!retVal) {
		// Failed to send out async request.
		return (false);
	}

	// Request is out and will be handled at some point by USB thread, wait on that.
	struct timespec waitForCompletionSleep = {.tv_sec = 0, .tv_nsec = 100000};

	while (!atomic_load(&dataCompletion.completed)) {
		// Sleep for 100µs to avoid busy loop.
		thrd_sleep(&waitForCompletionSleep, NULL);
	}

	if (atomic_load(&dataCompletion.completed) == 1) {
		// Success.
		return (true);
	}
	else {
		// Failure.
		return (false);
	}
}

static void syncControlOutCallback(void *controlOutCallbackPtr, int status) {
	atomic_uint_fast32_t *completed = controlOutCallbackPtr;

	if (status == LIBUSB_TRANSFER_COMPLETED) {
		atomic_store(completed, 1);
	}
	else {
		atomic_store(completed, 2);
	}
}

static void syncControlInCallback(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize) {
	usbDataCompletion dataCompletion = controlInCallbackPtr;

	if ((status == LIBUSB_TRANSFER_COMPLETED) && (bufferSize == dataCompletion->dataSize)) {
		// Copy data to location given by user.
		memcpy(dataCompletion->data, buffer, dataCompletion->dataSize);

		atomic_store(&dataCompletion->completed, 1);
	}
	else {
		atomic_store(&dataCompletion->completed, 2);
	}
}
