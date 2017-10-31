#ifndef LIBCAER_SRC_CONTAINER_GENERATION_H_
#define LIBCAER_SRC_CONTAINER_GENERATION_H_

#include "libcaer.h"
#include "data_exchange.h"
#include "timestamps.h"
#include "events/special.h"

struct container_generation {
	caerEventPacketContainer currentPacketContainer;
	atomic_uint_fast32_t maxPacketContainerPacketSize;
	atomic_uint_fast32_t maxPacketContainerInterval;
	int64_t currentPacketContainerCommitTimestamp;
};

typedef struct container_generation *containerGeneration;

static inline void containerGenerationSettingsInit(containerGeneration state) {
	// Packet settings (size (in events) and time interval (in µs)).
	// By default governed by time only, set at 10 milliseconds.
	atomic_store(&state->maxPacketContainerPacketSize, 0);
	atomic_store(&state->maxPacketContainerInterval, 10000);
}

static inline void containerGenerationDestroy(containerGeneration state) {
	if (state->currentPacketContainer != NULL) {
		caerEventPacketContainerFree(state->currentPacketContainer);
		state->currentPacketContainer = NULL;
	}
}

static inline void containerGenerationSetPacket(containerGeneration state, int32_t pos, caerEventPacketHeader packet) {
	if (state->currentPacketContainer != NULL) {
		caerEventPacketContainerSetEventPacket(state->currentPacketContainer, pos, packet);
	}
}

static inline bool containerGenerationAllocate(containerGeneration state, int32_t eventPacketNumber) {
	if (state->currentPacketContainer == NULL) {
		// Allocate packets.
		state->currentPacketContainer = caerEventPacketContainerAllocate(eventPacketNumber);
		if (state->currentPacketContainer == NULL) {
			return (false);
		}
	}

	return (true);
}

static inline int32_t containerGenerationGetMaxPacketSize(containerGeneration state) {
	return (I32T(atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed)));
}

static inline int32_t containerGenerationGetMaxInterval(containerGeneration state) {
	return (I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)));
}

static inline int64_t containerGenerationIsCommitTimestampElapsed(containerGeneration state, int32_t tsWrapOverflow,
	int32_t tsCurrent) {
	return (generateFullTimestamp(tsWrapOverflow, tsCurrent) > state->currentPacketContainerCommitTimestamp);
}

static inline void containerGenerationCommitTimestampReset(containerGeneration state) {
	// Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
	// will then set this correctly.
	state->currentPacketContainerCommitTimestamp = -1;
}

static inline void containerGenerationCommitTimestampInit(containerGeneration state, int32_t currentTimestamp) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = currentTimestamp + containerGenerationGetMaxInterval(state) - 1;
	}
}

static inline void containerGenerationExecute(containerGeneration state, bool emptyContainerCommit, bool tsReset,
	int32_t tsWrapOverflow, int32_t tsCurrent, dataExchange dataState, atomic_bool *transfersRunning, int16_t deviceId,
	const char *deviceString, atomic_uint_fast8_t *deviceLogLevelAtomic) {
	uint8_t deviceLogLevel = atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed);

	// If the commit was triggered by a packet container limit being reached, we always
	// update the time related limit. The size related one is updated implicitly by size
	// being reset to zero after commit (new packets are empty).
	if (containerGenerationIsCommitTimestampElapsed(state, tsWrapOverflow, tsCurrent)) {
		while (containerGenerationIsCommitTimestampElapsed(state, tsWrapOverflow, tsCurrent)) {
			state->currentPacketContainerCommitTimestamp += containerGenerationGetMaxInterval(state);
		}
	}

	// Filter out completely empty commits. This can happen when data is turned off,
	// but the timestamps are still going forward.
	if (emptyContainerCommit) {
		caerEventPacketContainerFree(state->currentPacketContainer);
		state->currentPacketContainer = NULL;
	}
	else {
		if (!dataExchangePut(dataState, state->currentPacketContainer)) {
			// Failed to forward packet container, just drop it, it doesn't contain
			// any critical information anyway.
			commonLog(CAER_LOG_NOTICE, deviceString, deviceLogLevel,
				"Dropped EventPacket Container because ring-buffer full!");

			caerEventPacketContainerFree(state->currentPacketContainer);
		}

		state->currentPacketContainer = NULL;
	}

	// The only critical timestamp information to forward is the timestamp reset event.
	// The timestamp big-wrap can also (and should!) be detected by observing a packet's
	// tsOverflow value, not the special packet TIMESTAMP_WRAP event, which is only informative.
	// For the timestamp reset event (TIMESTAMP_RESET), we thus ensure that it is always
	// committed, and we send it alone, in its own packet container, to ensure it will always
	// be ordered after any other event packets in any processing or output stream.
	if (tsReset) {
		// Allocate packet container just for this event.
		caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(1);
		if (tsResetContainer == NULL) {
			commonLog(CAER_LOG_CRITICAL, deviceString, deviceLogLevel,
				"Failed to allocate tsReset event packet container.");
			return;
		}

		// Allocate special packet just for this event.
		caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, deviceId, tsWrapOverflow);
		if (tsResetPacket == NULL) {
			commonLog(CAER_LOG_CRITICAL, deviceString, deviceLogLevel,
				"Failed to allocate tsReset special event packet.");
			return;
		}

		// Create timestamp reset event.
		caerSpecialEvent tsResetEvent = caerSpecialEventPacketGetEvent(tsResetPacket, 0);
		caerSpecialEventSetTimestamp(tsResetEvent, INT32_MAX);
		caerSpecialEventSetType(tsResetEvent, TIMESTAMP_RESET);
		caerSpecialEventValidate(tsResetEvent, tsResetPacket);

		// Assign special packet to packet container.
		caerEventPacketContainerSetEventPacket(tsResetContainer, SPECIAL_EVENT, (caerEventPacketHeader) tsResetPacket);

		// Reset MUST be committed, always, else downstream data processing and
		// outputs get confused if they have no notification of timestamps
		// jumping back go zero.
		dataExchangePutForce(dataState, transfersRunning, tsResetContainer);
	}
}

static inline bool containerGenerationConfigSet(containerGeneration state, uint8_t paramAddr, uint32_t param) {
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

	return (true);
}

static inline bool containerGenerationConfigGet(containerGeneration state, uint8_t paramAddr, uint32_t *param) {
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

	return (true);
}

#endif /* LIBCAER_SRC_CONTAINER_GENERATION_H_ */
