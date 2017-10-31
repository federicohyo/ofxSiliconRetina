#ifndef LIBCAER_SRC_TIMESTAMPS_H_
#define LIBCAER_SRC_TIMESTAMPS_H_

#include "libcaer.h"
#include "events/common.h"
#include "portable_time.h"
#include <stdatomic.h>

#ifdef NDEBUG
#define TIMESTAMPS_DEBUG 0
#else
#define TIMESTAMPS_DEBUG 1
#endif

#define TIMESTAMPS_DEBUG_DRIFT_ALARM 100000 // in µs, default 100ms.

struct timestamps_state_new_logic {
	int32_t wrapOverflow;
	int32_t wrapAdd;
	int32_t last;
	int32_t current;
#if defined(TIMESTAMPS_DEBUG) && TIMESTAMPS_DEBUG == 1
	bool debugInitialized;
	int64_t debugStartTimestamp;
	struct timespec debugStartTime;
#endif
};

typedef struct timestamps_state_new_logic *timestampsStateNewLogic;

static inline void commonLog(enum caer_log_level logLevel, const char *deviceString, uint8_t deviceLogLevel,
	const char *format, ...) ATTRIBUTE_FORMAT(4);

static inline void commonLog(enum caer_log_level logLevel, const char *deviceString, uint8_t deviceLogLevel,
	const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(), deviceLogLevel, logLevel,
		deviceString, format, argumentList);
	va_end(argumentList);
}

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T(U64T(U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void checkStrictMonotonicTimestamp(int32_t tsCurrent, int32_t tsLast, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	if (tsCurrent <= tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

static inline void checkMonotonicTimestamp(int32_t tsCurrent, int32_t tsLast, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	if (tsCurrent < tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamps: non monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

#if defined(TIMESTAMPS_DEBUG) && TIMESTAMPS_DEBUG == 1
static inline void timestampsDebugInit(timestampsStateNewLogic timestamps, bool reset) {
	if ((timestamps->debugInitialized == false) || (reset == true)) {
		timestamps->debugInitialized = true;

		timestamps->debugStartTimestamp = generateFullTimestamp(timestamps->wrapOverflow, timestamps->current);

		portable_clock_gettime_monotonic(&timestamps->debugStartTime);
	}
}
#endif

static inline bool handleTimestampWrapNewLogic(timestampsStateNewLogic timestamps, uint16_t wrapData, uint32_t wrapAdd,
	const char *deviceString, atomic_uint_fast8_t *deviceLogLevelAtomic) {
	// Detect big timestamp wrap-around.
	int64_t wrapJump = (wrapAdd * wrapData);
	int64_t wrapSum = I64T(timestamps->wrapAdd) + wrapJump;
	bool bigWrap = false;

	if (wrapSum > I64T(INT32_MAX)) {
		// Reset wrapAdd at this point, so we can again
		// start detecting overruns of the 32bit value.
		// We reset not to zero, but to the remaining value after
		// multiple wrap-jumps are taken into account.
		int64_t wrapRemainder = wrapSum - I64T(INT32_MAX) - 1LL;
		timestamps->wrapAdd = I32T(wrapRemainder);

		timestamps->last = 0;
		timestamps->current = timestamps->wrapAdd;

		// Increment TSOverflow counter.
		timestamps->wrapOverflow++;

		// Commit packets to separate before big wrap from after cleanly.
		bigWrap = true;
	}
	else {
		// Each wrap is 2^15 µs (~32ms), and we have
		// to multiply it with the wrap counter,
		// which is located in the data part of this
		// event.
		timestamps->wrapAdd = I32T(wrapSum);

		timestamps->last = timestamps->current;
		timestamps->current = timestamps->wrapAdd;

		// Check monotonicity of timestamps.
		checkStrictMonotonicTimestamp(timestamps->current, timestamps->last, deviceString, deviceLogLevelAtomic);

		commonLog(CAER_LOG_DEBUG, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamp wrap event received with multiplier of %" PRIu16 ".", wrapData);
	}

#if defined(TIMESTAMPS_DEBUG) && TIMESTAMPS_DEBUG == 1
	timestampsDebugInit(timestamps, false);

	// Timestamps debug mode: check elapsed system time (absolute monotonic time)
	// against elapsed time on device. They should roughly align and not drift over time.
	int64_t currentTimestamp = generateFullTimestamp(timestamps->wrapOverflow, timestamps->current);
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);

	int64_t timestampDifferenceMicro = currentTimestamp - timestamps->debugStartTimestamp;

	int64_t timeDifferenceNano = (I64T(currentTime.tv_sec - timestamps->debugStartTime.tv_sec) * 1000000000LL)
		+ I64T(currentTime.tv_nsec - timestamps->debugStartTime.tv_nsec);
	int64_t timeDifferenceMicro = timeDifferenceNano / 1000;

	int64_t tsDrift = llabs(timeDifferenceMicro - timestampDifferenceMicro);
	if (tsDrift >= TIMESTAMPS_DEBUG_DRIFT_ALARM) {
		commonLog(CAER_LOG_ERROR, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamps on host and device are drifting away, current drift is: %" PRIi64 " µs.", tsDrift);
	}
#endif

	return (bigWrap);
}

static inline void handleTimestampUpdateNewLogic(timestampsStateNewLogic timestamps, uint16_t tsData,
	const char *deviceString, atomic_uint_fast8_t *deviceLogLevelAtomic) {
	// Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
	timestamps->last = timestamps->current;
	timestamps->current = timestamps->wrapAdd + (tsData & 0x7FFF);

	// Check monotonicity of timestamps.
	checkStrictMonotonicTimestamp(timestamps->current, timestamps->last, deviceString, deviceLogLevelAtomic);

#if defined(TIMESTAMPS_DEBUG) && TIMESTAMPS_DEBUG == 1
	timestampsDebugInit(timestamps, false);
#endif
}

static inline void handleTimestampResetNewLogic(timestampsStateNewLogic timestamps, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	timestamps->wrapOverflow = 0;
	timestamps->wrapAdd = 0;
	timestamps->last = 0;
	timestamps->current = 0;

	commonLog(CAER_LOG_INFO, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
		"Timestamp reset event received.");

#if defined(TIMESTAMPS_DEBUG) && TIMESTAMPS_DEBUG == 1
	timestampsDebugInit(timestamps, true);
#endif
}

#endif /* LIBCAER_SRC_TIMESTAMPS_H_ */
