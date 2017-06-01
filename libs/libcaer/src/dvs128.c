#include "dvs128.h"

static void dvs128Log(enum caer_log_level logLevel, dvs128Handle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static void dvs128EventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent);
static bool dvs128SendBiases(dvs128State state);

static void dvs128Log(enum caer_log_level logLevel, dvs128Handle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static inline void checkMonotonicTimestamp(dvs128Handle handle) {
	if (handle->state.currentTimestamp < handle->state.lastTimestamp) {
		dvs128Log(CAER_LOG_ALERT, handle,
			"Timestamps: non monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void freeAllDataMemory(dvs128State state) {
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

	if (state->currentPacketContainer != NULL) {
		caerEventPacketContainerFree(state->currentPacketContainer);
		state->currentPacketContainer = NULL;
	}
}

caerDeviceHandle dvs128Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict) {
	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DVS_DEVICE_NAME);

	dvs128Handle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DVS128;

	dvs128State state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	atomic_store(&state->dataExchangeBufferSize, 64);
	atomic_store(&state->dataExchangeBlocking, false);
	atomic_store(&state->dataExchangeStartProducers, true);
	atomic_store(&state->dataExchangeStopProducers, true);

	// Packet settings (size (in events) and time interval (in µs)).
	atomic_store(&state->maxPacketContainerPacketSize, 4096);
	atomic_store(&state->maxPacketContainerInterval, 10000);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	atomic_store(&state->usbState.usbLogLevel, globalLogLevel);

	// Always master by default.
	atomic_store(&state->dvsIsMaster, true);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, DVS_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);

	// Try to open a DVS128 device on a specific USB port.
	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DVS_DEVICE_PID, busNumberRestrict, devAddressRestrict,
		serialNumberRestrict, -1, DVS_REQUIRED_FIRMWARE_VERSION)) {
		free(handle);

		return (NULL);
	}

	struct usb_info usbInfo = usbGenerateInfo(&state->usbState, DVS_DEVICE_NAME, deviceID);
	if (usbInfo.deviceString == NULL) {
		usbDeviceClose(&state->usbState);
		free(handle);

		return (NULL);
	}

	// Setup USB.
	usbSetDataCallback(&state->usbState, &dvs128EventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, DVS_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 4096);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);

		free(usbInfo.deviceString);
		free(handle);

		return (NULL);
	}

	// Populate info variables based on data from device.
	handle->info.deviceID = I16T(deviceID);
	strncpy(handle->info.deviceSerialNumber, usbInfo.serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);
	handle->info.deviceUSBBusNumber = usbInfo.busNumber;
	handle->info.deviceUSBDeviceAddress = usbInfo.devAddress;
	handle->info.deviceString = usbInfo.deviceString;
	handle->info.logicVersion = 1; // TODO: real logic revision, once that information is exposed by new logic.
	handle->info.deviceIsMaster = true;
	handle->info.dvsSizeX = DVS_ARRAY_SIZE_X;
	handle->info.dvsSizeY = DVS_ARRAY_SIZE_Y;

	dvs128Log(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		usbInfo.busNumber, usbInfo.devAddress);

	return ((caerDeviceHandle) handle);
}

bool dvs128Close(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	dvs128Log(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	dvs128Log(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	return (true);
}

struct caer_dvs128_info caerDVS128InfoGet(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_dvs128_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DVS128) {
		struct caer_dvs128_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool dvs128SendDefaultConfig(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	// Set all biases to default value. Based on DVS128 Fast biases.
	caerIntegerToByteArray(1992, state->biases[DVS128_CONFIG_BIAS_CAS], BIAS_LENGTH);
	caerIntegerToByteArray(1108364, state->biases[DVS128_CONFIG_BIAS_INJGND], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->biases[DVS128_CONFIG_BIAS_REQPD], BIAS_LENGTH);
	caerIntegerToByteArray(8159221, state->biases[DVS128_CONFIG_BIAS_PUX], BIAS_LENGTH);
	caerIntegerToByteArray(132, state->biases[DVS128_CONFIG_BIAS_DIFFOFF], BIAS_LENGTH);
	caerIntegerToByteArray(309590, state->biases[DVS128_CONFIG_BIAS_REQ], BIAS_LENGTH);
	caerIntegerToByteArray(969, state->biases[DVS128_CONFIG_BIAS_REFR], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->biases[DVS128_CONFIG_BIAS_PUY], BIAS_LENGTH);
	caerIntegerToByteArray(209996, state->biases[DVS128_CONFIG_BIAS_DIFFON], BIAS_LENGTH);
	caerIntegerToByteArray(13125, state->biases[DVS128_CONFIG_BIAS_DIFF], BIAS_LENGTH);
	caerIntegerToByteArray(271, state->biases[DVS128_CONFIG_BIAS_FOLL], BIAS_LENGTH);
	caerIntegerToByteArray(217, state->biases[DVS128_CONFIG_BIAS_PR], BIAS_LENGTH);

	// Send biases to device.
	return (dvs128SendBiases(state));
}

bool dvs128ConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

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

		case DVS128_CONFIG_DVS:
			switch (paramAddr) {
				case DVS128_CONFIG_DVS_RUN:
					if (param && !atomic_load(&state->dvsRunning)) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0)) {
							return (false);
						}

						atomic_store(&state->dvsRunning, true);
					}
					else if (!param && atomic_load(&state->dvsRunning)) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_STOP_TRANSFER, 0, 0, NULL, 0)) {
							return (false);
						}

						atomic_store(&state->dvsRunning, false);
					}
					break;

				case DVS128_CONFIG_DVS_TIMESTAMP_RESET:
					if (param) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_RESET_TS, 0, 0, NULL, 0)) {
							return (false);
						}
					}
					break;

				case DVS128_CONFIG_DVS_ARRAY_RESET:
					if (param) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_RESET_ARRAY, 0, 0, NULL, 0)) {
							return (false);
						}
					}
					break;

				case DVS128_CONFIG_DVS_TS_MASTER:
					if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_TS_MASTER, (param & 0x01), 0, NULL,
						0)) {
						return (false);
					}
					atomic_store(&state->dvsIsMaster, (param & 0x01));

					// Ensure info struct also gets this update.
					atomic_thread_fence(memory_order_seq_cst);
					handle->info.deviceIsMaster = atomic_load(&state->dvsIsMaster);
					atomic_thread_fence(memory_order_seq_cst);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS128_CONFIG_BIAS:
			switch (paramAddr) {
				case DVS128_CONFIG_BIAS_CAS:
				case DVS128_CONFIG_BIAS_INJGND:
				case DVS128_CONFIG_BIAS_PUX:
				case DVS128_CONFIG_BIAS_PUY:
				case DVS128_CONFIG_BIAS_REQPD:
				case DVS128_CONFIG_BIAS_REQ:
				case DVS128_CONFIG_BIAS_FOLL:
				case DVS128_CONFIG_BIAS_PR:
				case DVS128_CONFIG_BIAS_REFR:
				case DVS128_CONFIG_BIAS_DIFF:
				case DVS128_CONFIG_BIAS_DIFFON:
				case DVS128_CONFIG_BIAS_DIFFOFF:
					caerIntegerToByteArray(param, state->biases[paramAddr], BIAS_LENGTH);
					return (dvs128SendBiases(state));
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

bool dvs128ConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

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

		case DVS128_CONFIG_DVS:
			switch (paramAddr) {
				case DVS128_CONFIG_DVS_RUN:
					*param = atomic_load(&state->dvsRunning);
					break;

				case DVS128_CONFIG_DVS_TIMESTAMP_RESET:
				case DVS128_CONFIG_DVS_ARRAY_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DVS128_CONFIG_DVS_TS_MASTER:
					*param = atomic_load(&state->dvsIsMaster);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS128_CONFIG_BIAS:
			switch (paramAddr) {
				case DVS128_CONFIG_BIAS_CAS:
				case DVS128_CONFIG_BIAS_INJGND:
				case DVS128_CONFIG_BIAS_PUX:
				case DVS128_CONFIG_BIAS_PUY:
				case DVS128_CONFIG_BIAS_REQPD:
				case DVS128_CONFIG_BIAS_REQ:
				case DVS128_CONFIG_BIAS_FOLL:
				case DVS128_CONFIG_BIAS_PR:
				case DVS128_CONFIG_BIAS_REFR:
				case DVS128_CONFIG_BIAS_DIFF:
				case DVS128_CONFIG_BIAS_DIFFON:
				case DVS128_CONFIG_BIAS_DIFFOFF:
					*param = caerByteArrayToInteger(state->biases[paramAddr], BIAS_LENGTH);
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

bool dvs128DataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
	void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

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
		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(DVS_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPolarityPacket = caerPolarityEventPacketAllocate(DVS_POLARITY_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPolarityPacket == NULL) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(DVS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 6.
		dvs128ConfigSet((caerDeviceHandle) handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, true);
	}

	return (true);
}

bool dvs128DataStop(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 6.
		dvs128ConfigSet((caerDeviceHandle) handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, false);
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

	return (true);
}

// Remember to properly free the returned memory after usage!
caerEventPacketContainer dvs128DataGet(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;
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

#define DVS128_TIMESTAMP_WRAP_MASK 0x80
#define DVS128_TIMESTAMP_RESET_MASK 0x40
#define DVS128_POLARITY_SHIFT 0
#define DVS128_POLARITY_MASK 0x0001
#define DVS128_Y_ADDR_SHIFT 8
#define DVS128_Y_ADDR_MASK 0x007F
#define DVS128_X_ADDR_SHIFT 1
#define DVS128_X_ADDR_MASK 0x007F
#define DVS128_SYNC_EVENT_MASK 0x8000
#define TS_WRAP_ADD 0x4000

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T((U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void initContainerCommitTimestamp(dvs128State state) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = state->currentTimestamp
			+ I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
	}
}

static void dvs128EventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
	dvs128Handle handle = vhd;
	dvs128State state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bytesSent & 0x03) != 0) {
		dvs128Log(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of four.", bytesSent);
		bytesSent &= (size_t) ~0x03;
	}

	for (size_t i = 0; i < bytesSent; i += 4) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(DVS_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentPolarityPacket == NULL) {
			state->currentPolarityPacket = caerPolarityEventPacketAllocate(DVS_POLARITY_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentPolarityPacket == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
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
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to grow polarity event packet.");
				return;
			}

			state->currentPolarityPacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(DVS_SPECIAL_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
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
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentSpecialPacket = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		if ((buffer[i + 3] & DVS128_TIMESTAMP_WRAP_MASK) == DVS128_TIMESTAMP_WRAP_MASK) {
			// Detect big timestamp wrap-around.
			if (state->wrapAdd == (INT32_MAX - (TS_WRAP_ADD - 1))) {
				// Reset wrapAdd to zero at this point, so we can again
				// start detecting overruns of the 32bit value.
				state->wrapAdd = 0;

				state->lastTimestamp = 0;
				state->currentTimestamp = 0;

				// Increment TSOverflow counter.
				state->wrapOverflow++;

				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
					state->currentSpecialPacketPosition++);
				caerSpecialEventSetTimestamp(currentEvent, INT32_MAX);
				caerSpecialEventSetType(currentEvent, TIMESTAMP_WRAP);
				caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);

				// Commit packets to separate before wrap from after cleanly.
				tsBigWrap = true;
			}
			else {
				// timestamp bit 15 is one -> wrap: now we need to increment
				// the wrapAdd, uses only 14 bit timestamps. Each wrap is 2^14 µs (~16ms).
				state->wrapAdd += TS_WRAP_ADD;

				state->lastTimestamp = state->currentTimestamp;
				state->currentTimestamp = state->wrapAdd;
				initContainerCommitTimestamp(state);

				// Check monotonicity of timestamps.
				checkMonotonicTimestamp(handle);
			}
		}
		else if ((buffer[i + 3] & DVS128_TIMESTAMP_RESET_MASK) == DVS128_TIMESTAMP_RESET_MASK) {
			// timestamp bit 14 is one -> wrapAdd reset: this firmware
			// version uses reset events to reset timestamps
			state->wrapOverflow = 0;
			state->wrapAdd = 0;
			state->lastTimestamp = 0;
			state->currentTimestamp = 0;
			state->currentPacketContainerCommitTimestamp = -1;
			initContainerCommitTimestamp(state);

			// Defer timestamp reset event to later, so we commit it
			// alone, in its own packet.
			// Commit packets when doing a reset to clearly separate them.
			tsReset = true;
		}
		else {
			// address is LSB MSB (USB is LE)
			uint16_t addressUSB = le16toh(*((uint16_t * ) (&buffer[i])));

			// same for timestamp, LSB MSB (USB is LE)
			// 15 bit value of timestamp in 1 us tick
			uint16_t timestampUSB = le16toh(*((uint16_t * ) (&buffer[i + 2])));

			// Expand to 32 bits. (Tick is 1µs already.)
			state->lastTimestamp = state->currentTimestamp;
			state->currentTimestamp = state->wrapAdd + timestampUSB;
			initContainerCommitTimestamp(state);

			// Check monotonicity of timestamps.
			checkMonotonicTimestamp(handle);

			if ((addressUSB & DVS128_SYNC_EVENT_MASK) != 0) {
				// Special Trigger Event (MSB is set)
				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
					state->currentSpecialPacketPosition++);
				caerSpecialEventSetTimestamp(currentEvent, state->currentTimestamp);
				caerSpecialEventSetType(currentEvent, EXTERNAL_INPUT_RISING_EDGE);
				caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);
			}
			else {
				// Invert X values (flip along X axis). To correct for flipped camera.
				uint16_t x = U16T(
					(DVS_ARRAY_SIZE_X - 1) - U16T((addressUSB >> DVS128_X_ADDR_SHIFT) & DVS128_X_ADDR_MASK));
				// Invert Y values (flip along Y axis). To convert to CG format.
				uint16_t y = U16T(
					(DVS_ARRAY_SIZE_Y - 1) - U16T((addressUSB >> DVS128_Y_ADDR_SHIFT) & DVS128_Y_ADDR_MASK));
				// Invert polarity bit. Hardware is like this.
				bool polarity = (((addressUSB >> DVS128_POLARITY_SHIFT) & DVS128_POLARITY_MASK) == 0) ? (1) : (0);

				// Check range conformity.
				if (x >= DVS_ARRAY_SIZE_X) {
					dvs128Log(CAER_LOG_ALERT, handle, "X address out of range (0-%d): %" PRIu16 ".",
					DVS_ARRAY_SIZE_X - 1, x);
					continue; // Skip invalid event.
				}
				if (y >= DVS_ARRAY_SIZE_Y) {
					dvs128Log(CAER_LOG_ALERT, handle, "Y address out of range (0-%d): %" PRIu16 ".",
					DVS_ARRAY_SIZE_Y - 1, y);
					continue; // Skip invalid event.
				}

				caerPolarityEvent currentEvent = caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
					state->currentPolarityPacketPosition++);
				caerPolarityEventSetTimestamp(currentEvent, state->currentTimestamp);
				caerPolarityEventSetPolarity(currentEvent, polarity);
				caerPolarityEventSetY(currentEvent, y);
				caerPolarityEventSetX(currentEvent, x);
				caerPolarityEventValidate(currentEvent, state->currentPolarityPacket);
			}
		}

		// Thresholds on which to trigger packet container commit.
		// forceCommit is already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = I32T(
			atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentPolarityPacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize));

		bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
			> state->currentPacketContainerCommitTimestamp;

		// FIXME: with the current DVS128 architecture, currentTimestamp always comes together
		// with an event, so the very first event that matches this threshold will be
		// also part of the committed packet container. This doesn't break any of the invariants.

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
					dvs128Log(CAER_LOG_NOTICE, handle, "Dropped EventPacket Container because ring-buffer full!");

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
				caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(DVS_EVENT_TYPES);
				if (tsResetContainer == NULL) {
					dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset event packet container.");
					return;
				}

				// Allocate special packet just for this event.
				caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
					state->wrapOverflow);
				if (tsResetPacket == NULL) {
					dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset special event packet.");
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

static bool dvs128SendBiases(dvs128State state) {
	// Biases are already stored in an array with the same format as expected by
	// the device, we can thus send it directly.
	return (usbControlTransferOut(&state->usbState, VENDOR_REQUEST_SEND_BIASES, 0, 0, (uint8_t *) state->biases,
		(BIAS_NUMBER * BIAS_LENGTH)));
}
