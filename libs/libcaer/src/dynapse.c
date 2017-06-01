#include "dynapse.h"

static void dynapseLog(enum caer_log_level logLevel, dynapseHandle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static void dynapseEventTranslator(void *vdh, uint8_t *buffer, size_t bytesSent);

static void dynapseLog(enum caer_log_level logLevel, dynapseHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

// i = index, x = amount of columns, y = amount of rows
static inline uint32_t dynapseCalculateCoordinatesNeuX(uint32_t index, uint32_t columns, uint32_t rows) {
	//for each row
	for (size_t i = 0; i < rows; i++) {
		//check if the index parameter is in the row
		if (index < (columns * i) + columns && index >= columns * i) {
			//return x, y
			return (U32T(index - columns * i));
		}
	}

	return (0);
}

static inline uint32_t dynapseCalculateCoordinatesNeuY(uint32_t index, uint32_t columns, uint32_t rows) {
	//for each row
	for (size_t i = 0; i < rows; i++) {
		//check if the index parameter is in the row
		if (index < (columns * i) + columns && index >= columns * i) {
			//return x, y
			return (U32T(i));
		}
	}

	return (0);
}

// columns = amount of columns, x = column, y = row
//static inline uint32_t dynapseCalculateIndexNeu(uint32_t columns, uint32_t x, uint32_t y) {
//	return ((y * columns) + x);
//}

static inline void checkStrictMonotonicTimestamp(dynapseHandle handle) {
	if (handle->state.currentTimestamp <= handle->state.lastTimestamp) {
		dynapseLog(CAER_LOG_ALERT, handle,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void freeAllDataMemory(dynapseState state) {
	if (state->dataExchangeBuffer != NULL) {
		ringBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentSpikePacket != NULL) {
		free(&state->currentSpikePacket->packetHeader);
		state->currentSpikePacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DYNAPSE_SPIKE_EVENT_POS,
			NULL);
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

caerDeviceHandle dynapseOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict) {
	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DYNAPSE_DEVICE_NAME);

	dynapseHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DYNAPSE;

	dynapseState state = &handle->state;

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
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, DYNAPSE_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);

	// Try to open a Dynap-se device on a specific USB port.
	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DYNAPSE_DEVICE_PID, busNumberRestrict,
		devAddressRestrict, serialNumberRestrict, DYNAPSE_REQUIRED_LOGIC_REVISION, DYNAPSE_REQUIRED_FIRMWARE_VERSION)) {
		free(handle);

		return (NULL);
	}

	struct usb_info usbInfo = usbGenerateInfo(&state->usbState, DYNAPSE_DEVICE_NAME, deviceID);
	if (usbInfo.deviceString == NULL) {
		usbDeviceClose(&state->usbState);
		free(handle);

		return (NULL);
	}

	// Setup USB.
	usbSetDataCallback(&state->usbState, &dynapseEventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);

		free(usbInfo.deviceString);
		free(handle);

		return (NULL);
	}

	// Populate info variables based on data from device.
	uint32_t param32 = 0;

	handle->info.deviceID = I16T(deviceID);
	strncpy(handle->info.deviceSerialNumber, usbInfo.serialNumber, 8 + 1);
	handle->info.deviceUSBBusNumber = usbInfo.busNumber;
	handle->info.deviceUSBDeviceAddress = usbInfo.devAddress;
	handle->info.deviceString = usbInfo.deviceString;
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION, &param32);
	handle->info.logicVersion = I16T(param32);
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	handle->info.logicClock = I16T(param32);
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
	handle->info.chipID = I16T(param32);

	dynapseLog(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		usbInfo.busNumber, usbInfo.devAddress);

	return ((caerDeviceHandle) handle);
}

bool dynapseClose(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	dynapseLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	dynapseLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	return (true);
}

struct caer_dynapse_info caerDynapseInfoGet(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_dynapse_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		struct caer_dynapse_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool dynapseSendDefaultConfig(caerDeviceHandle cdh) {
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL, false);

	// TODO: on next logic update, this will switch to be in cycles, not 125µs blocks.
	// So will need to multiply by: 125 * 30 (FX2_USB_CLOCK_FREQ).
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs time-slices (defaults to 1ms)

	return (true);
}

bool dynapseConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

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

		case DYNAPSE_CONFIG_SRAM:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SYNAPSERECONFIG:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SYNAPSERECONFIG, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SPIKEGEN:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SPIKEGEN, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_MUX:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_MUX_RUN:
				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN:
				case DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL:
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_MUX, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						uint8_t spiMultiConfig[6 + 6] = { 0 };

						spiMultiConfig[0] = DYNAPSE_CONFIG_MUX;
						spiMultiConfig[1] = DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[2] = 0x00;
						spiMultiConfig[3] = 0x00;
						spiMultiConfig[4] = 0x00;
						spiMultiConfig[5] = 0x01;

						spiMultiConfig[6] = DYNAPSE_CONFIG_MUX;
						spiMultiConfig[7] = DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET;
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

		case DYNAPSE_CONFIG_AER:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_AER_RUN:
				case DYNAPSE_CONFIG_AER_ACK_DELAY:
				case DYNAPSE_CONFIG_AER_ACK_EXTENSION:
				case DYNAPSE_CONFIG_AER_WAIT_ON_TRANSFER_STALL:
				case DYNAPSE_CONFIG_AER_EXTERNAL_AER_CONTROL:
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_AER, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CHIP:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_CHIP_RUN:
				case DYNAPSE_CONFIG_CHIP_ID:
				case DYNAPSE_CONFIG_CHIP_REQ_DELAY:
				case DYNAPSE_CONFIG_CHIP_REQ_EXTENSION:
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_CHIP, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_CHIP_CONTENT: {
					uint8_t spiConfig[4] = { 0 };

					spiConfig[0] = U8T(param >> 24);
					spiConfig[1] = U8T(param >> 16);
					spiConfig[2] = U8T(param >> 8);
					spiConfig[3] = U8T(param >> 0);

					if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER, DYNAPSE_CONFIG_CHIP,
					DYNAPSE_CONFIG_CHIP_CONTENT, spiConfig, sizeof(spiConfig))) {
						dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to send chip config, USB transfer failed.");
						return (false);
					}

					uint8_t check[2] = { 0 };
					bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER, 0, 0, check,
						sizeof(check));
					if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER || check[1] != 0) {
						dynapseLog(CAER_LOG_CRITICAL, handle,
							"Failed to send chip config, USB transfer failed on verification.");
						return (false);
					}

					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DYNAPSE_CONFIG_USB:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_USB_RUN:
				case DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_USB, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CLEAR_CAM: {
			uint8_t spiMultiConfig[DYNAPSE_CONFIG_NUMCORES * DYNAPSE_CONFIG_NUMNEURONS * 6] = { 0 };

			size_t numConfig = 0;

			// all cores
			for (size_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
				// all rows
				for (size_t row = 0; row < DYNAPSE_CONFIG_NUMNEURONS; row++) {
					uint32_t bits = U32T(row << 5 | core << 15 | 1 << 17);

					spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
					spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
					spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
					spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
					spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
					spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

					numConfig++;
				}
			}

			size_t idxConfig = 0;

			while (numConfig > 0) {
				size_t configNum = (numConfig > 85) ? (85) : (numConfig);
				size_t configSize = configNum * 6;

				if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, U16T(configNum),
					0, spiMultiConfig + idxConfig, U16T(configSize))) {
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to clear CAM, USB transfer failed.");
					return (false);
				}

				uint8_t check[2] = { 0 };
				bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0, 0,
					check, sizeof(check));

				if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to clear CAM, USB transfer failed on verification.");
					return (false);
				}

				numConfig -= configNum;
				idxConfig += configSize;
			}

			break;
		}

		case DYNAPSE_CONFIG_MONITOR_NEU: {
			uint32_t neuid = param;
			uint32_t coreid = paramAddr;
			uint32_t bits = 0;
			uint32_t col, row;

			uint8_t spiMultiConfig[6 + 6] = { 0 };  // first reset then set

			uint32_t reset = coreid << 8 | 1 << 11 | 0 << 12 | 0 << 13 | 0 << 16 | 0 << 17; // reset monitor for this core

			if (neuid >= DYNAPSE_CONFIG_NUMNEURONS_CORE) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to monitor neuron, neuron id: %d is invalid.", neuid);
				return (false);
			}
			else {
				col = dynapseCalculateCoordinatesNeuX(neuid, DYNAPSE_CONFIG_NEUROW, DYNAPSE_CONFIG_NEUCOL);
				row = dynapseCalculateCoordinatesNeuY(neuid, DYNAPSE_CONFIG_NEUROW, DYNAPSE_CONFIG_NEUCOL);
				dynapseLog(CAER_LOG_NOTICE, handle, "Neuron ID %d results in neu at col: %d row: %d.", neuid, col, row);
				bits = coreid << 8 | 0 << 10 | 0 << 11 | 0 << 12 | 0 << 13 | 0 << 16 | 0 << 17 | row << 4 | col;

			}

			// organize data for USB send
			spiMultiConfig[0] = DYNAPSE_CONFIG_CHIP;
			spiMultiConfig[1] = DYNAPSE_CONFIG_CHIP_CONTENT;
			spiMultiConfig[2] = U8T((reset >> 24) & 0x0FF);
			spiMultiConfig[3] = U8T((reset >> 16) & 0x0FF);
			spiMultiConfig[4] = U8T((reset >> 8) & 0x0FF);
			spiMultiConfig[5] = U8T((reset >> 0) & 0x0FF);

			spiMultiConfig[6] = DYNAPSE_CONFIG_CHIP;
			spiMultiConfig[7] = DYNAPSE_CONFIG_CHIP_CONTENT;
			spiMultiConfig[8] = U8T((bits >> 24) & 0x0FF);
			spiMultiConfig[9] = U8T((bits >> 16) & 0x0FF);
			spiMultiConfig[10] = U8T((bits >> 8) & 0x0FF);
			spiMultiConfig[11] = U8T((bits >> 0) & 0x0FF);

			//usb send
			if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 2, 0, spiMultiConfig,
				sizeof(spiMultiConfig))) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to monitor neuron, USB transfer failed.");
				return (false);
			}

			uint8_t check[2] = { 0 };
			bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0, 0, check,
				sizeof(check));

			if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to monitor neuron, USB transfer failed on verification.");
				return (false);
			}

			break;
		}

		case DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY: {
			uint8_t spiMultiConfig[DYNAPSE_CONFIG_NUMCORES * DYNAPSE_CONFIG_SRAMROW * DYNAPSE_CONFIG_NUMSRAM_NEU * 6] =
				{ 0 }; // 6 pieces made of 8 bytes each

			size_t numConfig = 0;

			// route output neurons differently depending on the position on the board
			switch (paramAddr) {
				case DYNAPSE_CONFIG_DYNAPSE_U0:
				case DYNAPSE_CONFIG_DYNAPSE_U1:
				case DYNAPSE_CONFIG_DYNAPSE_U2:
				case DYNAPSE_CONFIG_DYNAPSE_U3:
					// route all neurons to the output south interface with
					// source chip id equal to DYNAPSE_CONFIG_DYNAPSE_U2
					// all cores
					for (uint32_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
						// all rows
						for (uint32_t row_neuronid = 0; row_neuronid < DYNAPSE_CONFIG_SRAMROW; row_neuronid++) {
							for (uint32_t row_sram = 0; row_sram < DYNAPSE_CONFIG_NUMSRAM_NEU; row_sram++) {

								uint32_t bits = row_neuronid << 5 | core << 15 | 1 << 17 | row_sram | 1 << 4; // init content to zero

								// organize data for USB send
								spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
								spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
								spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

								numConfig++;
							}
						}
					}

					size_t idxConfig = 0;

					while (numConfig > 0) {
						size_t configNum = (numConfig > 85) ? (85) : (numConfig);
						size_t configSize = configNum * 6;

						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE,
							U16T(configNum), 0, spiMultiConfig + idxConfig, U16T(configSize))) {
							dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to clear SRAM, USB transfer failed.");
							return (false);
						}

						uint8_t check[2] = { 0 };
						bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0,
							0, check, sizeof(check));

						if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
							dynapseLog(CAER_LOG_CRITICAL, handle,
								"Failed to clear SRAM, USB transfer failed on verification.");
							return (false);
						}

						numConfig -= configNum;
						idxConfig += configSize;
					}
			}

			break;
		}

		case DYNAPSE_CONFIG_DEFAULT_SRAM: {
			uint8_t spiMultiConfig[DYNAPSE_CONFIG_NUMCORES * DYNAPSE_CONFIG_NUMNEURONS_CORE * DYNAPSE_CONFIG_NUMSRAM_NEU
				* 6] = { 0 }; // 6 pieces made of 8 bytes each

			size_t numConfig = 0;
			size_t idxConfig = 0;

			uint32_t bits;
			uint32_t sourcechipid;
			uint32_t sign;
			uint32_t distance;
			uint32_t dx;
			uint32_t dy;
			uint32_t sx;
			uint32_t sy;
			uint32_t sourcecoreid;

			// route output neurons differently depending on the position on the board
			switch (paramAddr) {
				case DYNAPSE_CONFIG_DYNAPSE_U0: {
					// route all neurons to the output south interface with
					// source chip id equal to DYNAPSE_CONFIG_DYNAPSE_U2
					// all cores
					for (uint32_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
						// all rows
						for (uint32_t row_neuronid = 0; row_neuronid < DYNAPSE_CONFIG_NUMNEURONS_CORE; row_neuronid++) {
							for (uint32_t row_sram = 0; row_sram < DYNAPSE_CONFIG_NUMSRAM_NEU; row_sram++) {
								// use first sram for monitoring
								if (row_sram == 0) {
									sourcechipid = DYNAPSE_CONFIG_DYNAPSE_U0 + 1; // same as chip id
									sign = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG; // -1
									distance = 2;
									sourcecoreid = core;

									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4
										| sourcechipid << 18 | sign << 27 | distance << 25 | sourcecoreid << 28; // init content chip id and set correct destinations for monitoring
								}
								else {
									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4; // init content to zero
								}

								// organize data for USB send
								spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
								spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
								spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

								numConfig++;
							}
						}
					}

					while (numConfig > 0) {
						size_t configNum = (numConfig > 85) ? (85) : (numConfig);
						size_t configSize = configNum * 6;

						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE,
							U16T(configNum), 0, spiMultiConfig + idxConfig, U16T(configSize))) {
							dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to set SRAM, USB transfer failed.");
							return (false);
						}

						uint8_t check[2] = { 0 };
						bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0,
							0, check, sizeof(check));

						if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
							dynapseLog(CAER_LOG_CRITICAL, handle,
								"Failed to set SRAM, USB transfer failed on verification.");
							return (false);
						}

						numConfig -= configNum;
						idxConfig += configSize;
					}

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U1: {
					// route all neurons to the output south interface with
					// source chip id equal to DYNAPSE_CONFIG_DYNAPSE_U2
					// all cores
					for (uint32_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
						// all rows
						for (uint32_t row_neuronid = 0; row_neuronid < DYNAPSE_CONFIG_NUMNEURONS_CORE; row_neuronid++) {
							for (uint32_t row_sram = 0; row_sram < DYNAPSE_CONFIG_NUMSRAM_NEU; row_sram++) {
								// use first sram for monitoring
								if (row_sram == 0) {
									sourcechipid = DYNAPSE_CONFIG_DYNAPSE_U1; // same as chip id
									sx = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG; // -1
									dx = 1; // ns
									dy = 2;
									sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
									sourcecoreid = core;

									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4
										| sourcechipid << 18 | sy << 27 | dy << 25 | sx << 24 | dx << 22
										| sourcecoreid << 28; // init content chip id and set correct destinations for monitoring
								}
								else {
									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4; // init content to zero
								}

								// organize data for USB send
								spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
								spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
								spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

								numConfig++;
							}
						}
					}

					while (numConfig > 0) {
						size_t configNum = (numConfig > 85) ? (85) : (numConfig);
						size_t configSize = configNum * 6;

						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE,
							U16T(configNum), 0, spiMultiConfig + idxConfig, U16T(configSize))) {
							dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to set SRAM, USB transfer failed.");
							return (false);
						}

						uint8_t check[2] = { 0 };
						bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0,
							0, check, sizeof(check));

						if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
							dynapseLog(CAER_LOG_CRITICAL, handle,
								"Failed to set SRAM, USB transfer failed on verification.");
							return (false);
						}

						numConfig -= configNum;
						idxConfig += configSize;
					}

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U2: {
					// route all neurons to the output south interface with
					// source chip id equal to DYNAPSE_CONFIG_DYNAPSE_U2
					// all cores
					for (uint32_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
						// all rows
						for (uint32_t row_neuronid = 0; row_neuronid < DYNAPSE_CONFIG_NUMNEURONS_CORE; row_neuronid++) {
							for (uint32_t row_sram = 0; row_sram < DYNAPSE_CONFIG_NUMSRAM_NEU; row_sram++) {
								// use first sram for monitoring
								if (row_sram == 0) {
									sourcechipid = DYNAPSE_CONFIG_DYNAPSE_U2; // same as chip id
									sign = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG; // -1
									distance = 1;
									sourcecoreid = core;

									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4
										| sourcechipid << 18 | sign << 27 | distance << 25 | sourcecoreid << 28; // init content chip id and set correct destinations for monitoring
								}
								else {
									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4; // init content to zero
								}

								// organize data for USB send
								spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
								spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
								spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

								numConfig++;
							}
						}
					}

					while (numConfig > 0) {
						size_t configNum = (numConfig > 85) ? (85) : (numConfig);
						size_t configSize = configNum * 6;

						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE,
							U16T(configNum), 0, spiMultiConfig + idxConfig, U16T(configSize))) {
							dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to set SRAM, USB transfer failed.");
							return (false);
						}

						uint8_t check[2] = { 0 };
						bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0,
							0, check, sizeof(check));

						if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
							dynapseLog(CAER_LOG_CRITICAL, handle,
								"Failed to set SRAM, USB transfer failed on verification.");
							return (false);
						}

						numConfig -= configNum;
						idxConfig += configSize;
					}

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U3: {
					// route all neurons to the output south interface with
					// source chip id equal to DYNAPSE_CONFIG_DYNAPSE_U3
					// all cores
					for (uint32_t core = 0; core < DYNAPSE_CONFIG_NUMCORES; core++) {
						// all rows
						for (uint32_t row_neuronid = 0; row_neuronid < DYNAPSE_CONFIG_NUMNEURONS_CORE; row_neuronid++) {
							for (uint32_t row_sram = 0; row_sram < DYNAPSE_CONFIG_NUMSRAM_NEU; row_sram++) {
								// use first sram for monitoring
								if (row_sram == 0) {
									sourcechipid = DYNAPSE_CONFIG_DYNAPSE_U3; // same as chip id
									sx = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG; // -1
									dx = 1; // ns
									dy = 1;
									sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
									sourcecoreid = core;

									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4
										| sourcechipid << 18 | sy << 27 | dy << 25 | sx << 24 | dx << 22
										| sourcecoreid << 28; // init content chip id and set correct destinations for monitoring
								}
								else {
									bits = row_neuronid << 7 | row_sram << 5 | core << 15 | 1 << 17 | 1 << 4; // init content to zero
								}

								// organize data for USB send
								spiMultiConfig[(numConfig * 6) + 0] = DYNAPSE_CONFIG_CHIP;
								spiMultiConfig[(numConfig * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
								spiMultiConfig[(numConfig * 6) + 2] = U8T((bits >> 24) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 3] = U8T((bits >> 16) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 4] = U8T((bits >> 8) & 0x0FF);
								spiMultiConfig[(numConfig * 6) + 5] = U8T((bits >> 0) & 0x0FF);

								numConfig++;
							}
						}
					}

					while (numConfig > 0) {
						size_t configNum = (numConfig > 85) ? (85) : (numConfig);
						size_t configSize = configNum * 6;

						if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE,
							U16T(configNum), 0, spiMultiConfig + idxConfig, U16T(configSize))) {
							dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to set SRAM, USB transfer failed.");
							return (false);
						}

						uint8_t check[2] = { 0 };
						bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0,
							0, check, sizeof(check));

						if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
							dynapseLog(CAER_LOG_CRITICAL, handle,
								"Failed to set SRAM, USB transfer failed on verification.");
							return (false);
						}

						numConfig -= configNum;
						idxConfig += configSize;
					}

					break;
				}
			}

			break;
		}

		default:
			return (false);
			break;
	}

	return (true);
}

bool dynapseConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

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

		case DYNAPSE_CONFIG_MUX:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_MUX_RUN:
				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN:
				case DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_MUX, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SRAM:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SRAM, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SYNAPSERECONFIG:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYNAPSERECONFIG, paramAddr, param));
			break;
		case DYNAPSE_CONFIG_SPIKEGEN:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SPIKEGEN, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_AER:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_AER_RUN:
				case DYNAPSE_CONFIG_AER_ACK_DELAY:
				case DYNAPSE_CONFIG_AER_ACK_EXTENSION:
				case DYNAPSE_CONFIG_AER_WAIT_ON_TRANSFER_STALL:
				case DYNAPSE_CONFIG_AER_EXTERNAL_AER_CONTROL:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_AER, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CHIP:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_CHIP_RUN:
				case DYNAPSE_CONFIG_CHIP_ID:
				case DYNAPSE_CONFIG_CHIP_CONTENT:
				case DYNAPSE_CONFIG_CHIP_REQ_DELAY:
				case DYNAPSE_CONFIG_CHIP_REQ_EXTENSION:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_CHIP, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SYSINFO:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION:
				case DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER:
				case DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER:
				case DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_USB:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_USB_RUN:
				case DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_USB, paramAddr, param));
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

bool dynapseDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

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
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentSpikePacket = caerSpikeEventPacketAllocate(DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSpikePacket == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate spike event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(DYNAPSE_SPECIAL_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 2.
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, true);
	}

	return (true);
}

bool dynapseDataStop(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false); // Ensure chip turns off.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, false); // Turn off timestamping too.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, false);
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
	state->currentSpikePacketPosition = 0;
	state->currentSpecialPacketPosition = 0;

	return (true);
}

caerEventPacketContainer dynapseDataGet(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;
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

static inline void initContainerCommitTimestamp(dynapseState state) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = state->currentTimestamp
			+ I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
	}
}

static void dynapseEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
	dynapseHandle handle = vhd;
	dynapseState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		dynapseLog(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of two.", bytesSent);
		bytesSent &= (size_t) ~0x01;
	}

	for (size_t i = 0; i < bytesSent; i += 2) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(
			DYNAPSE_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentSpikePacket == NULL) {
			state->currentSpikePacket = caerSpikeEventPacketAllocate(
			DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpikePacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate spike event packet.");
				return;
			}
		}
		else if (state->currentSpikePacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpikePacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpikeEventPacket grownPacket = (caerSpikeEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentSpikePacket, state->currentSpikePacketPosition * 2);
			if (grownPacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to grow spike event packet.");
				return;
			}

			state->currentSpikePacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(
			DYNAPSE_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
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
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentSpecialPacket = grownPacket;
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
							dynapseLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							state->wrapOverflow = 0;
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;
							state->currentPacketContainerCommitTimestamp = -1;
							initContainerCommitTimestamp(state);

							dynapseLog(CAER_LOG_INFO, handle, "Timestamp reset event received.");

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							break;
						}

						default:
							dynapseLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.", data);
							break;
					}
					break;

				case 1: // AER addresses of Spikes.
				case 2: // Special encoding over 4 cases.
				case 5:
				case 6: {
					uint8_t sourceCoreID = 0; // code == 1

					if (code == 2) {
						sourceCoreID = 1;
					}
					else if (code == 5) {
						sourceCoreID = 2;
					}
					else if (code == 6) {
						sourceCoreID = 3;
					}

					uint8_t chipID = data & 0x0F;
					uint32_t neuronID = (data >> 4) & 0x00FF;

					caerSpikeEvent currentSpikeEvent = caerSpikeEventPacketGetEvent(state->currentSpikePacket,
						state->currentSpikePacketPosition);

					// Timestamp at event-stream insertion point.
					caerSpikeEventSetTimestamp(currentSpikeEvent, state->currentTimestamp);
					caerSpikeEventSetSourceCoreID(currentSpikeEvent, sourceCoreID);
					caerSpikeEventSetChipID(currentSpikeEvent, chipID);
					caerSpikeEventSetNeuronID(currentSpikeEvent, neuronID);
					caerSpikeEventValidate(currentSpikeEvent, state->currentSpikePacket);
					state->currentSpikePacketPosition++;

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

						dynapseLog(CAER_LOG_DEBUG, handle,
							"Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					}

					break;
				}

				default:
					dynapseLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// forceCommit is already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = I32T(
			atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentSpikePacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize));

		bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
			> state->currentPacketContainerCommitTimestamp;

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentSpikePacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DYNAPSE_SPIKE_EVENT_POS,
					(caerEventPacketHeader) state->currentSpikePacket);

				state->currentSpikePacket = NULL;
				state->currentSpikePacketPosition = 0;
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
					dynapseLog(CAER_LOG_NOTICE, handle, "Dropped EventPacket Container because ring-buffer full!");

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
				caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
				if (tsResetContainer == NULL) {
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset event packet container.");
					return;
				}

				// Allocate special packet just for this event.
				caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
					state->wrapOverflow);
				if (tsResetPacket == NULL) {
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset special event packet.");
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

bool caerDynapseSendDataToUSB(caerDeviceHandle cdh, const uint32_t *pointer, size_t numConfig) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	dynapseState state = &handle->state;

	// if array exceeds max size don't send anything
	if (numConfig > DYNAPSE_MAX_USER_USB_PACKET_SIZE) {
		return (false);
	}

	uint8_t spiMultiConfig[DYNAPSE_MAX_USER_USB_PACKET_SIZE * 6] = { 0 };

	// all cores
	for (size_t i = 0; i < numConfig; i++) {
		spiMultiConfig[(i * 6) + 0] = DYNAPSE_CONFIG_CHIP;
		spiMultiConfig[(i * 6) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
		spiMultiConfig[(i * 6) + 2] = U8T((pointer[i] >> 24) & 0x0FF);
		spiMultiConfig[(i * 6) + 3] = U8T((pointer[i] >> 16) & 0x0FF);
		spiMultiConfig[(i * 6) + 4] = U8T((pointer[i] >> 8) & 0x0FF);
		spiMultiConfig[(i * 6) + 5] = U8T((pointer[i] >> 0) & 0x0FF);
	}

	size_t idxConfig = 0;

	while (numConfig > 0) {
		size_t configNum = (numConfig > 85) ? (85) : (numConfig);
		size_t configSize = configNum * 6;

		if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, U16T(configNum), 0,
			spiMultiConfig + idxConfig, U16T(configSize))) {
			dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to clear CAM, USB transfer failed.");
			return (false);
		}

		uint8_t check[2] = { 0 };
		bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0, 0, check,
			sizeof(check));

		if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
			dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to clear CAM, USB transfer failed on verification.");
			return (false);
		}

		numConfig -= configNum;
		idxConfig += configSize;
	}

	// return true
	return (true);
}

bool caerDynapseWriteSramWords(caerDeviceHandle cdh, const uint16_t *data, uint32_t baseAddr, uint32_t numWords) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	dynapseState state = &handle->state;

	size_t numConfig = 0;
	// Handle even and odd numbers of words to write
	if (numWords % 2 == 0) {
		numConfig = numWords / 2;
	}
	else {
		// Handle the case where we have 1 trailing word
		// by just writing it manually
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_WRITEDATA, data[numWords - 1]);
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_ADDRESS, baseAddr + (numWords - 1));

		// reduce numWords to the, now even, number of remaining words.
		// Otherwise the spiMultiConfig array filling loop will be incorrect
		numWords--;

		// return if there was only 1 word to write
		if (numWords == 0) {
			return (true);
		}
		numConfig = numWords / 2;

	}

	// We need malloc because allocating dynamically sized arrays on the stack is not allowed.
	uint8_t *spiMultiConfig = malloc(numConfig * 6 * sizeof(*spiMultiConfig));

	if (spiMultiConfig == NULL) {
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to malloc spiMultiConfigArray");
		return (false); // No memory allocated, don't need to free.
	}

	for (size_t i = 0; i < numConfig; i++) {
		// Data word configuration
		spiMultiConfig[i * 6 + 0] = DYNAPSE_CONFIG_SRAM;
		spiMultiConfig[i * 6 + 1] = DYNAPSE_CONFIG_SRAM_WRITEDATA;
		spiMultiConfig[i * 6 + 2] = U8T((data[i * 2 + 1] >> 8) & 0x0FF);
		spiMultiConfig[i * 6 + 3] = U8T((data[i * 2 + 1] >> 0) & 0x0FF);
		spiMultiConfig[i * 6 + 4] = U8T((data[i * 2] >> 8) & 0x0FF);
		spiMultiConfig[i * 6 + 5] = U8T((data[i * 2] >> 0) & 0x0FF);
	}

	// Prepare the SRAM controller for writing
	// First we write the base address by writing a spoof word to it
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_WRITEDATA, 0x0);
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_ADDRESS, baseAddr);
	// Then we enable burst mode
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_BURSTMODE, 1);

	size_t idxConfig = 0;

	// Start writing data
	while (numConfig > 0) {
		size_t configNum = (numConfig > 85) ? (85) : (numConfig);
		size_t configSize = configNum * 6;

		if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, U16T(configNum), 0,
			spiMultiConfig + idxConfig, U16T(configSize))) {
			dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to send chip config, USB transfer failed.");

			free(spiMultiConfig);
			return (false);
		}

		numConfig -= configNum;
		idxConfig += configSize;
	}

	// Disable burst mode again or things will go wrong when accessing the SRAM in the future
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_BURSTMODE, 0);

	free(spiMultiConfig);

	return (true);
}

bool caerDynapseWriteCam(caerDeviceHandle cdh, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId,
	int16_t synapseType) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	uint32_t bits = caerDynapseGenerateCamBits(preNeuronAddr, postNeuronAddr, camId, synapseType);

	if (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits) == false) {
		return (false);
	}

	return (true);
}

uint32_t caerDynapseGenerateCamBits(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId,
	int16_t synapseType) {
	uint32_t ei = (U32T(synapseType) & 0x2) >> 1;
	uint32_t fs = synapseType & 0x1;
	uint32_t address = preNeuronAddr & 0xff;
	uint32_t source_core = (preNeuronAddr & 0x300) >> 8;
	uint32_t coreId = (postNeuronAddr & 0x300) >> 8;
	uint32_t neuron_row = (postNeuronAddr & 0xf0) >> 4;
	uint32_t synapse_row = camId;
	uint32_t row = neuron_row << 6 | synapse_row;
	uint32_t column = postNeuronAddr & 0xf;

	uint32_t bits = ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17 | coreId << 15 | row << 5
		| column;

	return (bits);
}

bool caerDynapseWriteSram(caerDeviceHandle cdh, uint16_t coreId, uint32_t neuronId, uint16_t virtualCoreId, bool sx,
	uint8_t dx, bool sy, uint8_t dy, uint16_t sramId, uint16_t destinationCore) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	uint32_t bits = neuronId
		<< 7| U32T(sramId << 5) | U32T(coreId << 15) | 1 << 17 | 1 << 4 | U32T(destinationCore << 18)
		| U32T(sy << 27) | U32T(dy << 25) | U32T(dx << 22) | U32T(sx << 24) | U32T(virtualCoreId << 28);

	if (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits) == false) {
		return (false);
	}

	return (true);
}
