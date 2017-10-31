#include "dvs128.h"

static void dvs128Log(enum caer_log_level logLevel, dvs128Handle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static void dvs128EventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent);
static bool dvs128SendBiases(dvs128State state);

static void dvs128Log(enum caer_log_level logLevel, dvs128Handle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static inline void freeAllDataMemory(dvs128State state) {
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

	containerGenerationDestroy(&state->container);
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
	dataExchangeSettingsInit(&state->dataExchange);

	// Packet settings (size (in events) and time interval (in µs)).
	containerGenerationSettingsInit(&state->container);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	atomic_store(&state->usbState.usbLogLevel, globalLogLevel);

	// Always master by default.
	atomic_store(&state->dvs.isMaster, true);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, DVS_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);
	handle->info.deviceString = usbThreadName; // Temporary, until replaced by full string.

	// Try to open a DVS128 device on a specific USB port.
	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DVS_DEVICE_PID, busNumberRestrict, devAddressRestrict,
		serialNumberRestrict, -1, DVS_REQUIRED_FIRMWARE_VERSION)) {
		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to open device.");
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
	handle->info.logicVersion = 1;
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

	dvs128Log(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

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
	caerIntegerToByteArray(1992, state->dvs.biases[DVS128_CONFIG_BIAS_CAS], BIAS_LENGTH);
	caerIntegerToByteArray(1108364, state->dvs.biases[DVS128_CONFIG_BIAS_INJGND], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->dvs.biases[DVS128_CONFIG_BIAS_REQPD], BIAS_LENGTH);
	caerIntegerToByteArray(8159221, state->dvs.biases[DVS128_CONFIG_BIAS_PUX], BIAS_LENGTH);
	caerIntegerToByteArray(132, state->dvs.biases[DVS128_CONFIG_BIAS_DIFFOFF], BIAS_LENGTH);
	caerIntegerToByteArray(309590, state->dvs.biases[DVS128_CONFIG_BIAS_REQ], BIAS_LENGTH);
	caerIntegerToByteArray(969, state->dvs.biases[DVS128_CONFIG_BIAS_REFR], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->dvs.biases[DVS128_CONFIG_BIAS_PUY], BIAS_LENGTH);
	caerIntegerToByteArray(209996, state->dvs.biases[DVS128_CONFIG_BIAS_DIFFON], BIAS_LENGTH);
	caerIntegerToByteArray(13125, state->dvs.biases[DVS128_CONFIG_BIAS_DIFF], BIAS_LENGTH);
	caerIntegerToByteArray(271, state->dvs.biases[DVS128_CONFIG_BIAS_FOLL], BIAS_LENGTH);
	caerIntegerToByteArray(217, state->dvs.biases[DVS128_CONFIG_BIAS_PR], BIAS_LENGTH);

	// Send biases to device.
	return (dvs128SendBiases(state));
}

bool dvs128ConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			return (usbConfigSet(&state->usbState, paramAddr, param));
			break;

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
					if ((param == 1) && (!atomic_load(&state->dvs.running))) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0)) {
							return (false);
						}

						atomic_store(&state->dvs.running, true);
					}
					else if ((param == 0) && atomic_load(&state->dvs.running)) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_STOP_TRANSFER, 0, 0, NULL, 0)) {
							return (false);
						}

						atomic_store(&state->dvs.running, false);
					}
					break;

				case DVS128_CONFIG_DVS_TIMESTAMP_RESET:
					if (param == 1) {
						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_RESET_TS, 0, 0, NULL, 0)) {
							return (false);
						}
					}
					break;

				case DVS128_CONFIG_DVS_ARRAY_RESET:
					if (param == 1) {
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
					atomic_store(&state->dvs.isMaster, (param & 0x01));

					// Ensure info struct also gets this update.
					atomic_thread_fence(memory_order_seq_cst);
					handle->info.deviceIsMaster = atomic_load(&state->dvs.isMaster);
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
					caerIntegerToByteArray(param, state->dvs.biases[paramAddr], BIAS_LENGTH);
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
			return (usbConfigGet(&state->usbState, paramAddr, param));
			break;

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

		case DVS128_CONFIG_DVS:
			switch (paramAddr) {
				case DVS128_CONFIG_DVS_RUN:
					*param = atomic_load(&state->dvs.running);
					break;

				case DVS128_CONFIG_DVS_TIMESTAMP_RESET:
				case DVS128_CONFIG_DVS_ARRAY_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DVS128_CONFIG_DVS_TS_MASTER:
					*param = atomic_load(&state->dvs.isMaster);
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
					*param = caerByteArrayToInteger(state->dvs.biases[paramAddr], BIAS_LENGTH);
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
	dataExchangeSetNotify(&state->dataExchange, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr);

	usbSetShutdownCallback(&state->usbState, dataShutdownNotify, dataShutdownUserPtr);

	containerGenerationCommitTimestampReset(&state->container);

	if (!dataExchangeBufferInit(&state->dataExchange)) {
		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	if (!containerGenerationAllocate(&state->container, DVS_EVENT_TYPES)) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPackets.polarity = caerPolarityEventPacketAllocate(DVS_POLARITY_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPackets.polarity == NULL) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentPackets.special = caerSpecialEventPacketAllocate(DVS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentPackets.special == NULL) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (dataExchangeStartProducers(&state->dataExchange)) {
		// Enable data transfer on USB end-point 6.
		dvs128ConfigSet((caerDeviceHandle) handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, true);
	}

	return (true);
}

bool dvs128DataStop(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	if (dataExchangeStopProducers(&state->dataExchange)) {
		// Disable data transfer on USB end-point 6.
		dvs128ConfigSet((caerDeviceHandle) handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_RUN, false);
	}

	usbDataTransfersStop(&state->usbState);

	dataExchangeBufferEmpty(&state->dataExchange);

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPackets.polarityPosition = 0;
	state->currentPackets.specialPosition = 0;

	return (true);
}

// Remember to properly free the returned memory after usage!
caerEventPacketContainer dvs128DataGet(caerDeviceHandle cdh) {
	dvs128Handle handle = (dvs128Handle) cdh;
	dvs128State state = &handle->state;

	return (dataExchangeGet(&state->dataExchange, &state->usbState.dataTransfersRun));
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

static void dvs128EventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent) {
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
		bytesSent &= ~((size_t) 0x03);
	}

	for (size_t i = 0; i < bytesSent; i += 4) {
		// Allocate new packets for next iteration as needed.
		if (!containerGenerationAllocate(&state->container, DVS_EVENT_TYPES)) {
			dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
			return;
		}

		if (state->currentPackets.polarity == NULL) {
			state->currentPackets.polarity = caerPolarityEventPacketAllocate(DVS_POLARITY_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.polarity == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}
		else if (state->currentPackets.polarityPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.polarity)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerPolarityEventPacket grownPacket = (caerPolarityEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.polarity, state->currentPackets.polarityPosition * 2);
			if (grownPacket == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to grow polarity event packet.");
				return;
			}

			state->currentPackets.polarity = grownPacket;
		}

		if (state->currentPackets.special == NULL) {
			state->currentPackets.special = caerSpecialEventPacketAllocate(DVS_SPECIAL_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.special == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}
		else if (state->currentPackets.specialPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.special)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.special, state->currentPackets.specialPosition * 2);
			if (grownPacket == NULL) {
				dvs128Log(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentPackets.special = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		if ((buffer[i + 3] & DVS128_TIMESTAMP_WRAP_MASK) == DVS128_TIMESTAMP_WRAP_MASK) {
			// Detect big timestamp wrap-around.
			if (state->timestamps.wrapAdd == (INT32_MAX - (TS_WRAP_ADD - 1))) {
				// Reset wrapAdd to zero at this point, so we can again
				// start detecting overruns of the 32bit value.
				state->timestamps.wrapAdd = 0;

				state->timestamps.last = 0;
				state->timestamps.current = 0;

				// Increment TSOverflow counter.
				state->timestamps.wrapOverflow++;

				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentPackets.special,
					state->currentPackets.specialPosition);
				state->currentPackets.specialPosition++;

				caerSpecialEventSetTimestamp(currentEvent, INT32_MAX);
				caerSpecialEventSetType(currentEvent, TIMESTAMP_WRAP);
				caerSpecialEventValidate(currentEvent, state->currentPackets.special);

				// Commit packets to separate before wrap from after cleanly.
				tsBigWrap = true;
			}
			else {
				// timestamp bit 15 is one -> wrap: now we need to increment
				// the wrapAdd, uses only 14 bit timestamps. Each wrap is 2^14 µs (~16ms).
				state->timestamps.wrapAdd += TS_WRAP_ADD;

				state->timestamps.last = state->timestamps.current;
				state->timestamps.current = state->timestamps.wrapAdd;
				containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

				// Check monotonicity of timestamps.
				checkMonotonicTimestamp(state->timestamps.current, state->timestamps.last,
					handle->info.deviceString, &handle->state.deviceLogLevel);
			}
		}
		else if ((buffer[i + 3] & DVS128_TIMESTAMP_RESET_MASK) == DVS128_TIMESTAMP_RESET_MASK) {
			// timestamp bit 14 is one -> wrapAdd reset: this firmware
			// version uses reset events to reset timestamps
			state->timestamps.wrapOverflow = 0;
			state->timestamps.wrapAdd = 0;
			state->timestamps.last = 0;
			state->timestamps.current = 0;
			containerGenerationCommitTimestampReset(&state->container);
			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

			// Defer timestamp reset event to later, so we commit it
			// alone, in its own packet.
			// Commit packets when doing a reset to clearly separate them.
			tsReset = true;
		}
		else {
			// address is LSB MSB (USB is LE)
			uint16_t addressUSB = le16toh(*((const uint16_t *) (&buffer[i])));

			// same for timestamp, LSB MSB (USB is LE)
			// 15 bit value of timestamp in 1 us tick
			uint16_t timestampUSB = le16toh(*((const uint16_t *) (&buffer[i + 2])));

			// Expand to 32 bits. (Tick is 1µs already.)
			state->timestamps.last = state->timestamps.current;
			state->timestamps.current = state->timestamps.wrapAdd + timestampUSB;
			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

			// Check monotonicity of timestamps.
			checkMonotonicTimestamp(state->timestamps.current, state->timestamps.last,
				handle->info.deviceString, &handle->state.deviceLogLevel);

			if ((addressUSB & DVS128_SYNC_EVENT_MASK) != 0) {
				// Special Trigger Event (MSB is set)
				caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentPackets.special,
					state->currentPackets.specialPosition);
				state->currentPackets.specialPosition++;

				caerSpecialEventSetTimestamp(currentEvent, state->timestamps.current);
				caerSpecialEventSetType(currentEvent, EXTERNAL_INPUT_RISING_EDGE);
				caerSpecialEventValidate(currentEvent, state->currentPackets.special);
			}
			else {
				// Invert X values (flip along X axis). To correct for flipped camera.
				uint16_t x = U16T(
					(DVS_ARRAY_SIZE_X - 1) - U16T((addressUSB >> DVS128_X_ADDR_SHIFT) & DVS128_X_ADDR_MASK));
				// Invert Y values (flip along Y axis). To convert to CG format.
				uint16_t y = U16T(
					(DVS_ARRAY_SIZE_Y - 1) - U16T((addressUSB >> DVS128_Y_ADDR_SHIFT) & DVS128_Y_ADDR_MASK));
				// Invert polarity bit. Hardware is like this.
				bool polarity = ((U16T(addressUSB >> DVS128_POLARITY_SHIFT) & DVS128_POLARITY_MASK) == 0) ? (1) : (0);

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

				caerPolarityEvent currentEvent = caerPolarityEventPacketGetEvent(state->currentPackets.polarity,
					state->currentPackets.polarityPosition);
				state->currentPackets.polarityPosition++;

				caerPolarityEventSetTimestamp(currentEvent, state->timestamps.current);
				caerPolarityEventSetPolarity(currentEvent, polarity);
				caerPolarityEventSetY(currentEvent, y);
				caerPolarityEventSetX(currentEvent, x);
				caerPolarityEventValidate(currentEvent, state->currentPackets.polarity);
			}
		}

		// Thresholds on which to trigger packet container commit.
		// tsReset and tsBigWrap are already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = containerGenerationGetMaxPacketSize(&state->container);
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentPackets.polarityPosition >= currentPacketContainerCommitSize)
				|| (state->currentPackets.specialPosition >= currentPacketContainerCommitSize));

		bool containerTimeCommit = containerGenerationIsCommitTimestampElapsed(&state->container,
			state->timestamps.wrapOverflow, state->timestamps.current);

		// NOTE: with the current DVS128 architecture, currentTimestamp always comes together
		// with an event, so the very first event that matches this threshold will be
		// also part of the committed packet container. This doesn't break any of the invariants.

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPackets.polarityPosition > 0) {
				containerGenerationSetPacket(&state->container, POLARITY_EVENT,
					(caerEventPacketHeader) state->currentPackets.polarity);

				state->currentPackets.polarity = NULL;
				state->currentPackets.polarityPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentPackets.specialPosition > 0) {
				containerGenerationSetPacket(&state->container, SPECIAL_EVENT,
					(caerEventPacketHeader) state->currentPackets.special);

				state->currentPackets.special = NULL;
				state->currentPackets.specialPosition = 0;
				emptyContainerCommit = false;
			}

			containerGenerationExecute(&state->container, emptyContainerCommit, tsReset, state->timestamps.wrapOverflow,
				state->timestamps.current, &state->dataExchange, &state->usbState.dataTransfersRun,
				handle->info.deviceID, handle->info.deviceString, &handle->state.deviceLogLevel);
		}
	}
}

static bool dvs128SendBiases(dvs128State state) {
	// Biases are already stored in an array with the same format as expected by
	// the device, we can thus send it directly.
	return (usbControlTransferOut(&state->usbState, VENDOR_REQUEST_SEND_BIASES, 0, 0, (uint8_t *) state->dvs.biases,
		(BIAS_NUMBER * BIAS_LENGTH)));
}
