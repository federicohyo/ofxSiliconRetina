/**
 * @file config.h
 *
 * Configuration Events format definition and handling functions.
 * This event contains information about the current configuration of
 * the device. By having configuration as a standardized event format,
 * it becomes host-software agnostic, and it also becomes part of the
 * event stream, enabling easy tracking of changes through time, by
 * putting them into the event stream at the moment they happen.
 * While the resolution of the timestamps for these events is in
 * microseconds for compatibility with all other event types, the
 * precision is in the order of ~1-20 milliseconds, given that these
 * events are generated and injected on the host-side.
 */

#ifndef LIBCAER_EVENTS_CONFIG_H_
#define LIBCAER_EVENTS_CONFIG_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Shift and mask values for the module address.
 * Module address is only 7 bits, since the eighth bit
 * is used device-side to differentiate reads from writes.
 * Here we can just re-use it for the validity mark.
 */
//@{
#define CONFIG_MODULE_ADDR_SHIFT 1
#define CONFIG_MODULE_ADDR_MASK 0x0000007F
//@}

/**
 * Configuration event data structure definition.
 * This contains the actual configuration module address, the
 * parameter address and the actual parameter content, as
 * well as the 32 bit event timestamp.
 * Signed integers are used for fields that are to be interpreted
 * directly, for compatibility with languages that do not have
 * unsigned integer types, such as Java.
 */
PACKED_STRUCT(struct caer_configuration_event {
	/// Configuration module address. First (also) because of valid mark.
	uint8_t moduleAddress;
	/// Configuration parameter address.
	uint8_t parameterAddress;
	/// Configuration parameter content (4 bytes).
	uint32_t parameter;
	/// Event timestamp.
	int32_t timestamp;
});

/**
 * Type for pointer to configuration event data structure.
 */
typedef struct caer_configuration_event *caerConfigurationEvent;
typedef const struct caer_configuration_event *caerConfigurationEventConst;

/**
 * Configuration event packet data structure definition.
 * EventPackets are always made up of the common packet header,
 * followed by 'eventCapacity' events. Everything has to
 * be in one contiguous memory block.
 */
PACKED_STRUCT(struct caer_configuration_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct caer_configuration_event events[];
});

/**
 * Type for pointer to configuration event packet data structure.
 */
typedef struct caer_configuration_event_packet *caerConfigurationEventPacket;
typedef const struct caer_configuration_event_packet *caerConfigurationEventPacketConst;

/**
 * Allocate a new configuration events packet.
 * Use free() to reclaim this memory.
 *
 * @param eventCapacity the maximum number of events this packet will hold.
 * @param eventSource the unique ID representing the source/generator of this packet.
 * @param tsOverflow the current timestamp overflow counter value for this packet.
 *
 * @return a valid ConfigurationEventPacket handle or NULL on error.
 */
static inline caerConfigurationEventPacket caerConfigurationEventPacketAllocate(
	int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
	return ((caerConfigurationEventPacket) caerEventPacketAllocate(eventCapacity, eventSource, tsOverflow, CONFIG_EVENT,
		sizeof(struct caer_configuration_event), offsetof(struct caer_configuration_event, timestamp)));
}

/**
 * Transform a generic event packet header into a Configuration event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid event packet header pointer. Cannot be NULL.
 * @return a properly converted, typed event packet pointer.
 */
static inline caerConfigurationEventPacket caerConfigurationEventPacketFromPacketHeader(caerEventPacketHeader header) {
	if (caerEventPacketHeaderGetEventType(header) != CONFIG_EVENT) {
		return (NULL);
	}

	return ((caerConfigurationEventPacket) header);
}

/**
 * Transform a generic read-only event packet header into a read-only Configuration event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid read-only event packet header pointer. Cannot be NULL.
 * @return a properly converted, read-only typed event packet pointer.
 */
static inline caerConfigurationEventPacketConst caerConfigurationEventPacketFromPacketHeaderConst(
	caerEventPacketHeaderConst header) {
	if (caerEventPacketHeaderGetEventType(header) != CONFIG_EVENT) {
		return (NULL);
	}

	return ((caerConfigurationEventPacketConst) header);
}

/**
 * Get the configuration event at the given index from the event packet.
 *
 * @param packet a valid ConfigurationEventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested configuration event. NULL on error.
 */
static inline caerConfigurationEvent caerConfigurationEventPacketGetEvent(
	caerConfigurationEventPacket packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLogEHO(CAER_LOG_CRITICAL, "Configuration Event",
			"Called caerConfigurationEventPacketGetEvent() with invalid event offset %" PRIi32
			", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Get the configuration event at the given index from the event packet.
 * This is a read-only event, do not change its contents in any way!
 *
 * @param packet a valid ConfigurationEventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested read-only configuration event. NULL on error.
 */
static inline caerConfigurationEventConst caerConfigurationEventPacketGetEventConst(
	caerConfigurationEventPacketConst packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLogEHO(CAER_LOG_CRITICAL, "Configuration Event",
			"Called caerConfigurationEventPacketGetEventConst() with invalid event offset %" PRIi32
			", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Get the 32bit event timestamp, in microseconds.
 * Be aware that this wraps around! You can either ignore this fact,
 * or handle the special 'TIMESTAMP_WRAP' event that is generated when
 * this happens, or use the 64bit timestamp which never wraps around.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 *
 * @return this event's 32bit microsecond timestamp.
 */
static inline int32_t caerConfigurationEventGetTimestamp(caerConfigurationEventConst event) {
	return (I32T(le32toh(U32T(event->timestamp))));
}

/**
 * Get the 64bit event timestamp, in microseconds.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param packet the ConfigurationEventPacket pointer for the packet containing this event. Cannot be NULL.
 *
 * @return this event's 64bit microsecond timestamp.
 */
static inline int64_t caerConfigurationEventGetTimestamp64(
	caerConfigurationEventConst event, caerConfigurationEventPacketConst packet) {
	return (I64T((U64T(caerEventPacketHeaderGetEventTSOverflow(&packet->packetHeader)) << TS_OVERFLOW_SHIFT)
				 | U64T(caerConfigurationEventGetTimestamp(event))));
}

/**
 * Set the 32bit event timestamp, the value has to be in microseconds.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param timestamp a positive 32bit microsecond timestamp.
 */
static inline void caerConfigurationEventSetTimestamp(caerConfigurationEvent event, int32_t timestamp) {
	if (timestamp < 0) {
		// Negative means using the 31st bit!
		caerLogEHO(CAER_LOG_CRITICAL, "Configuration Event",
			"Called caerConfigurationEventSetTimestamp() with negative value!");
		return;
	}

	event->timestamp = I32T(htole32(U32T(timestamp)));
}

/**
 * Check if this configuration event is valid.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 *
 * @return true if valid, false if not.
 */
static inline bool caerConfigurationEventIsValid(caerConfigurationEventConst event) {
	return (GET_NUMBITS8(event->moduleAddress, VALID_MARK_SHIFT, VALID_MARK_MASK));
}

/**
 * Validate the current event by setting its valid bit to true
 * and increasing the event packet's event count and valid
 * event count. Only works on events that are invalid.
 * DO NOT CALL THIS AFTER HAVING PREVIOUSLY ALREADY
 * INVALIDATED THIS EVENT, the total count will be incorrect.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param packet the ConfigurationEventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerConfigurationEventValidate(caerConfigurationEvent event, caerConfigurationEventPacket packet) {
	if (!caerConfigurationEventIsValid(event)) {
		SET_NUMBITS8(event->moduleAddress, VALID_MARK_SHIFT, VALID_MARK_MASK, 1);

		// Also increase number of events and valid events.
		// Only call this on (still) invalid events!
		caerEventPacketHeaderSetEventNumber(
			&packet->packetHeader, caerEventPacketHeaderGetEventNumber(&packet->packetHeader) + 1);
		caerEventPacketHeaderSetEventValid(
			&packet->packetHeader, caerEventPacketHeaderGetEventValid(&packet->packetHeader) + 1);
	}
	else {
		caerLogEHO(CAER_LOG_CRITICAL, "Configuration Event",
			"Called caerConfigurationEventValidate() on already valid event.");
	}
}

/**
 * Invalidate the current event by setting its valid bit
 * to false and decreasing the number of valid events held
 * in the packet. Only works with events that are already
 * valid!
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param packet the ConfigurationEventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerConfigurationEventInvalidate(caerConfigurationEvent event, caerConfigurationEventPacket packet) {
	if (caerConfigurationEventIsValid(event)) {
		CLEAR_NUMBITS8(event->moduleAddress, VALID_MARK_SHIFT, VALID_MARK_MASK);

		// Also decrease number of valid events. Number of total events doesn't change.
		// Only call this on valid events!
		caerEventPacketHeaderSetEventValid(
			&packet->packetHeader, caerEventPacketHeaderGetEventValid(&packet->packetHeader) - 1);
	}
	else {
		caerLogEHO(CAER_LOG_CRITICAL, "Configuration Event",
			"Called caerConfigurationEventInvalidate() on already invalid event.");
	}
}

/**
 * Get the configuration event's module address.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 *
 * @return configuration module address.
 */
static inline uint8_t caerConfigurationEventGetModuleAddress(caerConfigurationEventConst event) {
	return U8T(GET_NUMBITS8(event->moduleAddress, CONFIG_MODULE_ADDR_SHIFT, CONFIG_MODULE_ADDR_MASK));
}

/**
 * Set the configuration event's module address.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param moduleAddress configuration module address.
 */
static inline void caerConfigurationEventSetModuleAddress(caerConfigurationEvent event, uint8_t moduleAddress) {
	CLEAR_NUMBITS8(event->moduleAddress, CONFIG_MODULE_ADDR_SHIFT, CONFIG_MODULE_ADDR_MASK);
	SET_NUMBITS8(event->moduleAddress, CONFIG_MODULE_ADDR_SHIFT, CONFIG_MODULE_ADDR_MASK, moduleAddress);
}

/**
 * Get the configuration event's parameter address.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 *
 * @return configuration parameter address.
 */
static inline uint8_t caerConfigurationEventGetParameterAddress(caerConfigurationEventConst event) {
	return (event->parameterAddress);
}

/**
 * Set the configuration event's parameter address.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param parameterAddress configuration parameter address.
 */
static inline void caerConfigurationEventSetParameterAddress(caerConfigurationEvent event, uint8_t parameterAddress) {
	event->parameterAddress = parameterAddress;
}

/**
 * Get the configuration event's parameter.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 *
 * @return configuration parameter.
 */
static inline uint32_t caerConfigurationEventGetParameter(caerConfigurationEventConst event) {
	return (I32T(le32toh(event->parameter)));
}

/**
 * Set the configuration event's parameter.
 *
 * @param event a valid ConfigurationEvent pointer. Cannot be NULL.
 * @param parameter configuration parameter.
 */
static inline void caerConfigurationEventSetParameter(caerConfigurationEvent event, uint32_t parameter) {
	event->parameter = I32T(htole32(parameter));
}

/**
 * Iterator over all configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEvent.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_ITERATOR_ALL_START(CONFIGURATION_PACKET)                    \
	for (int32_t caerConfigurationIteratorCounter = 0;                                 \
		 caerConfigurationIteratorCounter                                              \
		 < caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader); \
		 caerConfigurationIteratorCounter++) {                                         \
		caerConfigurationEvent caerConfigurationIteratorElement                        \
			= caerConfigurationEventPacketGetEvent(CONFIGURATION_PACKET, caerConfigurationIteratorCounter);

/**
 * Const-Iterator over all configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEventConst.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_CONST_ITERATOR_ALL_START(CONFIGURATION_PACKET)              \
	for (int32_t caerConfigurationIteratorCounter = 0;                                 \
		 caerConfigurationIteratorCounter                                              \
		 < caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader); \
		 caerConfigurationIteratorCounter++) {                                         \
		caerConfigurationEventConst caerConfigurationIteratorElement                   \
			= caerConfigurationEventPacketGetEventConst(CONFIGURATION_PACKET, caerConfigurationIteratorCounter);

/**
 * Iterator close statement.
 */
#define CAER_CONFIGURATION_ITERATOR_ALL_END }

/**
 * Iterator over only the valid configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEvent.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_ITERATOR_VALID_START(CONFIGURATION_PACKET)                                       \
	for (int32_t caerConfigurationIteratorCounter = 0;                                                      \
		 caerConfigurationIteratorCounter                                                                   \
		 < caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader);                      \
		 caerConfigurationIteratorCounter++) {                                                              \
		caerConfigurationEvent caerConfigurationIteratorElement                                             \
			= caerConfigurationEventPacketGetEvent(CONFIGURATION_PACKET, caerConfigurationIteratorCounter); \
		if (!caerConfigurationEventIsValid(caerConfigurationIteratorElement)) {                             \
			continue;                                                                                       \
		} // Skip invalid configuration events.

/**
 * Const-Iterator over only the valid configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEventConst.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_CONST_ITERATOR_VALID_START(CONFIGURATION_PACKET)                                      \
	for (int32_t caerConfigurationIteratorCounter = 0;                                                           \
		 caerConfigurationIteratorCounter                                                                        \
		 < caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader);                           \
		 caerConfigurationIteratorCounter++) {                                                                   \
		caerConfigurationEventConst caerConfigurationIteratorElement                                             \
			= caerConfigurationEventPacketGetEventConst(CONFIGURATION_PACKET, caerConfigurationIteratorCounter); \
		if (!caerConfigurationEventIsValid(caerConfigurationIteratorElement)) {                                  \
			continue;                                                                                            \
		} // Skip invalid configuration events.

/**
 * Iterator close statement.
 */
#define CAER_CONFIGURATION_ITERATOR_VALID_END }

/**
 * Reverse iterator over all configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEvent.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_REVERSE_ITERATOR_ALL_START(CONFIGURATION_PACKET)                \
	for (int32_t caerConfigurationIteratorCounter                                          \
		 = caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader) - 1; \
		 caerConfigurationIteratorCounter >= 0; caerConfigurationIteratorCounter--) {      \
		caerConfigurationEvent caerConfigurationIteratorElement                            \
			= caerConfigurationEventPacketGetEvent(CONFIGURATION_PACKET, caerConfigurationIteratorCounter);
/**
 * Const-Reverse iterator over all configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEventConst.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_CONST_REVERSE_ITERATOR_ALL_START(CONFIGURATION_PACKET)          \
	for (int32_t caerConfigurationIteratorCounter                                          \
		 = caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader) - 1; \
		 caerConfigurationIteratorCounter >= 0; caerConfigurationIteratorCounter--) {      \
		caerConfigurationEventConst caerConfigurationIteratorElement                       \
			= caerConfigurationEventPacketGetEventConst(CONFIGURATION_PACKET, caerConfigurationIteratorCounter);

/**
 * Reverse iterator close statement.
 */
#define CAER_CONFIGURATION_REVERSE_ITERATOR_ALL_END }

/**
 * Reverse iterator over only the valid configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEvent.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_REVERSE_ITERATOR_VALID_START(CONFIGURATION_PACKET)                               \
	for (int32_t caerConfigurationIteratorCounter                                                           \
		 = caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader) - 1;                  \
		 caerConfigurationIteratorCounter >= 0; caerConfigurationIteratorCounter--) {                       \
		caerConfigurationEvent caerConfigurationIteratorElement                                             \
			= caerConfigurationEventPacketGetEvent(CONFIGURATION_PACKET, caerConfigurationIteratorCounter); \
		if (!caerConfigurationEventIsValid(caerConfigurationIteratorElement)) {                             \
			continue;                                                                                       \
		} // Skip invalid configuration events.

/**
 * Const-Reverse iterator over only the valid configuration events in a packet.
 * Returns the current index in the 'caerConfigurationIteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerConfigurationIteratorElement' variable
 * of type caerConfigurationEventConst.
 *
 * CONFIGURATION_PACKET: a valid ConfigurationEventPacket pointer. Cannot be NULL.
 */
#define CAER_CONFIGURATION_CONST_REVERSE_ITERATOR_VALID_START(CONFIGURATION_PACKET)                              \
	for (int32_t caerConfigurationIteratorCounter                                                                \
		 = caerEventPacketHeaderGetEventNumber(&(CONFIGURATION_PACKET)->packetHeader) - 1;                       \
		 caerConfigurationIteratorCounter >= 0; caerConfigurationIteratorCounter--) {                            \
		caerConfigurationEventConst caerConfigurationIteratorElement                                             \
			= caerConfigurationEventPacketGetEventConst(CONFIGURATION_PACKET, caerConfigurationIteratorCounter); \
		if (!caerConfigurationEventIsValid(caerConfigurationIteratorElement)) {                                  \
			continue;                                                                                            \
		} // Skip invalid configuration events.

/**
 * Reverse iterator close statement.
 */
#define CAER_CONFIGURATION_REVERSE_ITERATOR_VALID_END }

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_CONFIG_H_ */
