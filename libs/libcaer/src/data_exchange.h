#ifndef LIBCAER_SRC_DATA_EXCHANGE_H_
#define LIBCAER_SRC_DATA_EXCHANGE_H_

#include "libcaer.h"
#include "devices/device.h"
#include "ringbuffer.h"
#include <stdatomic.h>

#if defined(HAVE_PTHREADS)
#include "c11threads_posix.h"
#endif

struct data_exchange {
	caerRingBuffer buffer;
	atomic_uint_fast32_t bufferSize; // Only takes effect on DataStart() calls!
	atomic_bool blocking;
	atomic_bool startProducers;
	atomic_bool stopProducers;
	void (*notifyDataIncrease)(void *ptr);
	void (*notifyDataDecrease)(void *ptr);
	void *notifyDataUserPtr;
};

typedef struct data_exchange *dataExchange;

static inline void dataExchangeSettingsInit(dataExchange state) {
	atomic_store(&state->bufferSize, 64);
	atomic_store(&state->blocking, false);
	atomic_store(&state->startProducers, true);
	atomic_store(&state->stopProducers, true);
}

static inline bool dataExchangeBufferInit(dataExchange state) {
	// Initialize RingBuffer.
	state->buffer = caerRingBufferInit(atomic_load(&state->bufferSize));
	if (state->buffer == NULL) {
		return (false);
	}

	return (true);
}

static inline void dataExchangeDestroy(dataExchange state) {
	if (state->buffer != NULL) {
		caerRingBufferFree(state->buffer);
		state->buffer = NULL;
	}
}

static inline caerEventPacketContainer dataExchangeGet(dataExchange state, atomic_bool *transfersRunning) {
	caerEventPacketContainer container = NULL;

	retry: container = caerRingBufferGet(state->buffer);

	if (container != NULL) {
		// Found an event container, return it and signal this piece of data
		// is no longer available for later acquisition.
		if (state->notifyDataDecrease != NULL) {
			state->notifyDataDecrease(state->notifyDataUserPtr);
		}

		return (container);
	}

	// Didn't find any event container, either report this or retry, depending
	// on blocking setting.
	if (atomic_load_explicit(&state->blocking, memory_order_relaxed) && atomic_load(transfersRunning)) {
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

static inline bool dataExchangePut(dataExchange state, caerEventPacketContainer container) {
	if (!caerRingBufferPut(state->buffer, container)) {
		return (false);
	}
	else {
		if (state->notifyDataIncrease != NULL) {
			state->notifyDataIncrease(state->notifyDataUserPtr);
		}

		return (true);
	}
}

static inline void dataExchangePutForce(dataExchange state, atomic_bool *transfersRunning,
	caerEventPacketContainer container) {
	while (!caerRingBufferPut(state->buffer, container)) {
		// Prevent dead-lock if shutdown is requested and nothing is consuming
		// data anymore, but the ring-buffer is full (and would thus never empty),
		// thus blocking the USB handling thread in this loop.
		if (!atomic_load(transfersRunning)) {
			return;
		}
	}

	// Signal new container as usual.
	if (state->notifyDataIncrease != NULL) {
		state->notifyDataIncrease(state->notifyDataUserPtr);
	}
}

static inline void dataExchangeBufferEmpty(dataExchange state) {
	// Empty ringbuffer.
	caerEventPacketContainer container;
	while ((container = caerRingBufferGet(state->buffer)) != NULL) {
		// Notify data-not-available call-back.
		if (state->notifyDataDecrease != NULL) {
			state->notifyDataDecrease(state->notifyDataUserPtr);
		}

		// Free container, which will free its subordinate packets too.
		caerEventPacketContainerFree(container);
	}
}

static inline void dataExchangeSetNotify(dataExchange state, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr) {
	state->notifyDataIncrease = dataNotifyIncrease;
	state->notifyDataDecrease = dataNotifyDecrease;
	state->notifyDataUserPtr = dataNotifyUserPtr;
}

static inline bool dataExchangeStartProducers(dataExchange state) {
	return (atomic_load(&state->startProducers));
}

static inline bool dataExchangeStopProducers(dataExchange state) {
	return (atomic_load(&state->stopProducers));
}

static inline bool dataExchangeConfigSet(dataExchange state, uint8_t paramAddr, uint32_t param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
			atomic_store(&state->bufferSize, param);
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
			atomic_store(&state->blocking, param);
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
			atomic_store(&state->startProducers, param);
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
			atomic_store(&state->stopProducers, param);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

static inline bool dataExchangeConfigGet(dataExchange state, uint8_t paramAddr, uint32_t *param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
			*param = U32T(atomic_load(&state->bufferSize));
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
			*param = atomic_load(&state->blocking);
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
			*param = atomic_load(&state->startProducers);
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
			*param = atomic_load(&state->stopProducers);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

#endif /* LIBCAER_SRC_DATA_EXCHANGE_H_ */
