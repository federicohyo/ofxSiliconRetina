/**
 * @file matrix4x4.h
 *
 * THIS EVENT DEFINITION IS STILL TO BE CONSIDERED EXPERIMENTAL
 * AND IS SUBJECT TO FUTURE CHANGES AND REVISIONS!
 *
 * Matrix4x4 Events format definition and handling functions.
 * This contains a matrix of dimensions 4x4 with floats entries,
 * together with support for distinguishing type and scale.
 * Useful for homogeneous coordinates for example.
 *
 *   m00  m01  m02  m03
 *   m10  m11  m12  m13
 *   m20  m21  m22  m23
 *   m30  m31  m32  m33
 *
 */

#ifndef LIBCAER_EVENTS_MATRIX4x4_H_
#define LIBCAER_EVENTS_MATRIX4x4_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Shift and mask values for type and scale information
 * associated with a Matrix4x4 event.
 * Up to 128 types are supported. The scale is given as orders
 * of magnitude, from 10^-128 to 10^127.
 * Bit 0 is the valid mark, see 'common.h' for more details.
 */
//@{
#define MATRIX4x4_TYPE_SHIFT 1
#define MATRIX4x4_TYPE_MASK 0x0000007F
#define MATRIX4x4_SCALE_SHIFT 8
#define MATRIX4x4_SCALE_MASK 0x000000FF
//@}

/**
 * Matrix4x4 event data structure definition.
 * This contains information about the measurement, such as a type
 * and a scale field, together with the usual validity mark.
 * The measurements are stored as floats.
 *   m00  m01  m02  m03
 *   m10  m11  m12  m13
 *   m20  m21  m22  m23
 *   m30  m31  m32  m33
 * Floats are in IEEE 754-2008 binary32 format.
 * Signed integers are used for fields that are to be interpreted
 * directly, for compatibility with languages that do not have
 * unsigned integer types, such as Java.
 */
PACKED_STRUCT(
struct caer_matrix4x4_event {
	/// Event information. First because of valid mark.
	uint32_t info;
	/// m00 axis measurement.
	float m[4][4];
	/// Event timestamp.
	int32_t timestamp;
});

/**
 * Type for pointer to Matrix4x4 event data structure.
 */
typedef struct caer_matrix4x4_event *caerMatrix4x4Event;
typedef const struct caer_matrix4x4_event *caerMatrix4x4EventConst;

/**
 * Matrix4x4 event packet data structure definition.
 * EventPackets are always made up of the common packet header,
 * followed by 'eventCapacity' events. Everything has to
 * be in one contiguous memory block.
 */
PACKED_STRUCT(
struct caer_matrix4x4_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct caer_matrix4x4_event events[];
});

/**
 * Type for pointer to Matrix4x4 event packet data structure.
 */
typedef struct caer_matrix4x4_event_packet *caerMatrix4x4EventPacket;
typedef const struct caer_matrix4x4_event_packet *caerMatrix4x4EventPacketConst;

/**
 * Allocate a new Matrix4x4 events packet.
 * Use free() to reclaim this memory.
 *
 * @param eventCapacity the maximum number of events this packet will hold.
 * @param eventSource the unique ID representing the source/generator of this packet.
 * @param tsOverflow the current timestamp overflow counter value for this packet.
 *
 * @return a valid Matrix4x4EventPacket handle or NULL on error.
 */
caerMatrix4x4EventPacket caerMatrix4x4EventPacketAllocate(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow);

/**
 * Transform a generic event packet header into a Matrix4x4 event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid event packet header pointer. Cannot be NULL.
 * @return a properly converted, typed event packet pointer.
 */
static inline caerMatrix4x4EventPacket caerMatrix4x4EventPacketFromPacketHeader(caerEventPacketHeader header) {
	if (caerEventPacketHeaderGetEventType(header) != MATRIX4x4_EVENT) {
		return (NULL);
	}

	return ((caerMatrix4x4EventPacket) header);
}

/**
 * Transform a generic read-only event packet header into a read-only Matrix4x4 event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid read-only event packet header pointer. Cannot be NULL.
 * @return a properly converted, read-only typed event packet pointer.
 */
static inline caerMatrix4x4EventPacketConst caerMatrix4x4EventPacketFromPacketHeaderConst(caerEventPacketHeaderConst header) {
	if (caerEventPacketHeaderGetEventType(header) != MATRIX4x4_EVENT) {
		return (NULL);
	}

	return ((caerMatrix4x4EventPacketConst) header);
}

/**
 * Get the Matrix4x4 event at the given index from the event packet.
 *
 * @param packet a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested Matrix4x4 event. NULL on error.
 */
static inline caerMatrix4x4Event caerMatrix4x4EventPacketGetEvent(caerMatrix4x4EventPacket packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "Matrix4x4 Event",
			"Called caerMatrix4x4EventPacketGetEvent() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Get the Matrix4x4 event at the given index from the event packet.
 * This is a read-only event, do not change its contents in any way!
 *
 * @param packet a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested read-only Matrix4x4 event. NULL on error.
 */
static inline caerMatrix4x4EventConst caerMatrix4x4EventPacketGetEventConst(caerMatrix4x4EventPacketConst packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "Matrix4x4 Event",
			"Called caerMatrix4x4EventPacketGetEventConst() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ".",
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
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return this event's 32bit microsecond timestamp.
 */
static inline int32_t caerMatrix4x4EventGetTimestamp(caerMatrix4x4EventConst event) {
	return (le32toh(event->timestamp));
}

/**
 * Get the 64bit event timestamp, in microseconds.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param packet the Matrix4x4EventPacket pointer for the packet containing this event. Cannot be NULL.
 *
 * @return this event's 64bit microsecond timestamp.
 */
static inline int64_t caerMatrix4x4EventGetTimestamp64(caerMatrix4x4EventConst event, caerMatrix4x4EventPacketConst packet) {
	return (I64T(
		(U64T(caerEventPacketHeaderGetEventTSOverflow(&packet->packetHeader)) << TS_OVERFLOW_SHIFT) | U64T(caerMatrix4x4EventGetTimestamp(event))));
}

/**
 * Set the 32bit event timestamp, the value has to be in microseconds.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param timestamp a positive 32bit microsecond timestamp.
 */
static inline void caerMatrix4x4EventSetTimestamp(caerMatrix4x4Event event, int32_t timestamp) {
	if (timestamp < 0) {
		// Negative means using the 31st bit!
		caerLog(CAER_LOG_CRITICAL, "Matrix4x4 Event", "Called caerMatrix4x4EventSetTimestamp() with negative value!");
		return;
	}

	event->timestamp = htole32(timestamp);
}

/**
 * Check if this Matrix4x4 event is valid.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return true if valid, false if not.
 */
static inline bool caerMatrix4x4EventIsValid(caerMatrix4x4EventConst event) {
	return (GET_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK));
}

/**
 * Validate the current event by setting its valid bit to true
 * and increasing the event packet's event count and valid
 * event count. Only works on events that are invalid.
 * DO NOT CALL THIS AFTER HAVING PREVIOUSLY ALREADY
 * INVALIDATED THIS EVENT, the total count will be incorrect.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param packet the Matrix4x4EventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerMatrix4x4EventValidate(caerMatrix4x4Event event, caerMatrix4x4EventPacket packet) {
	if (!caerMatrix4x4EventIsValid(event)) {
		SET_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK, 1);

		// Also increase number of events and valid events.
		// Only call this on (still) invalid events!
		caerEventPacketHeaderSetEventNumber(&packet->packetHeader,
			caerEventPacketHeaderGetEventNumber(&packet->packetHeader) + 1);
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) + 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "Matrix4x4 Event", "Called caerMatrix4x4EventValidate() on already valid event.");
	}
}

/**
 * Invalidate the current event by setting its valid bit
 * to false and decreasing the number of valid events held
 * in the packet. Only works with events that are already
 * valid!
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param packet the Matrix4x4EventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerMatrix4x4EventInvalidate(caerMatrix4x4Event event, caerMatrix4x4EventPacket packet) {
	if (caerMatrix4x4EventIsValid(event)) {
		CLEAR_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK);

		// Also decrease number of valid events. Number of total events doesn't change.
		// Only call this on valid events!
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) - 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "Matrix4x4 Event", "Called caerMatrix4x4EventInvalidate() on already invalid event.");
	}
}

/**
 * Get the measurement event type. This is useful to distinguish
 * between different measurements, for example distance or weight.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return the Matrix4x4 measurement type.
 */
static inline uint8_t caerMatrix4x4EventGetType(caerMatrix4x4EventConst event) {
	return U8T(GET_NUMBITS32(event->info, MATRIX4x4_TYPE_SHIFT, MATRIX4x4_TYPE_MASK));
}

/**
 * Set the measurement event type. This is useful to distinguish
 * between different measurements, for example distance or weight.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param type the Matrix4x4 measurement type.
 */
static inline void caerMatrix4x4EventSetType(caerMatrix4x4Event event, uint8_t type) {
	CLEAR_NUMBITS32(event->info, MATRIX4x4_TYPE_SHIFT, MATRIX4x4_TYPE_MASK);
	SET_NUMBITS32(event->info, MATRIX4x4_TYPE_SHIFT, MATRIX4x4_TYPE_MASK, type);
}

/**
 * Get the measurement scale. This allows order of magnitude shifts
 * on the measured value to be applied automatically, such as having
 * measurements of type Distance (meters) and storing the values as
 * centimeters (10^-2) for higher precision, but keeping that information
 * around to allow easy changes of unit.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return the Matrix4x4 measurement scale.
 */
static inline int8_t caerMatrix4x4EventGetScale(caerMatrix4x4EventConst event) {
	return I8T(GET_NUMBITS32(event->info, MATRIX4x4_SCALE_SHIFT, MATRIX4x4_SCALE_MASK));
}

/**
 * Set the measurement scale. This allows order of magnitude shifts
 * on the measured value to be applied automatically, such as having
 * measurements of type Distance (meters) and storing the values as
 * centimeters (10^-2) for higher precision, but keeping that information
 * around to allow easy changes of unit.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param scale the Matrix4x4 measurement scale.
 */
static inline void caerMatrix4x4EventSetScale(caerMatrix4x4Event event, int8_t scale) {
	CLEAR_NUMBITS32(event->info, MATRIX4x4_SCALE_SHIFT, MATRIX4x4_SCALE_MASK);
	SET_NUMBITS32(event->info, MATRIX4x4_SCALE_SHIFT, MATRIX4x4_SCALE_MASK, U8T(scale));
}

/**
 * Get the M00 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M00 element.
 */
static inline float caerMatrix4x4EventGetM00(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[0][0]));
}

/**
 * Set the M00 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m00 value.
 */
static inline void caerMatrix4x4EventSetM00(caerMatrix4x4Event event, float x) {
	event->m[0][0] = htoleflt(x);
}

/**
 * Get the M01 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M01 element.
 */
static inline float caerMatrix4x4EventGetM01(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[0][1]));
}

/**
 * Set the M01 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m01 value.
 */
static inline void caerMatrix4x4EventSetM01(caerMatrix4x4Event event, float x) {
	event->m[0][1] = htoleflt(x);
}

/**
 * Get the M02 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M02 element.
 */
static inline float caerMatrix4x4EventGetM02(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[0][2]));
}

/**
 * Set the M02 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m02 value.
 */
static inline void caerMatrix4x4EventSetM02(caerMatrix4x4Event event, float x) {
	event->m[0][2] = htoleflt(x);
}

/**
 * Get the M03 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M01 element.
 */
static inline float caerMatrix4x4EventGetM03(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[0][3]));
}

/**
 * Set the M03 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m03 value.
 */
static inline void caerMatrix4x4EventSetM03(caerMatrix4x4Event event, float x) {
	event->m[0][3] = htoleflt(x);
}

/**
 * Get the M10 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M10 element.
 */
static inline float caerMatrix4x4EventGetM10(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[1][0]));
}

/**
 * Set the M10 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m10 value.
 */
static inline void caerMatrix4x4EventSetM10(caerMatrix4x4Event event, float x) {
	event->m[1][0] = htoleflt(x);
}

/**
 * Get the M11 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M11 element.
 */
static inline float caerMatrix4x4EventGetM11(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[1][1]));
}

/**
 * Set the M11 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m11 value.
 */
static inline void caerMatrix4x4EventSetM11(caerMatrix4x4Event event, float x) {
	event->m[1][1] = htoleflt(x);
}

/**
 * Get the M12 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M12 element.
 */
static inline float caerMatrix4x4EventGetM12(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[1][2]));
}

/**
 * Set the M12 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m12 value.
 */
static inline void caerMatrix4x4EventSetM12(caerMatrix4x4Event event, float x) {
	event->m[1][2] = htoleflt(x);
}

/**
 * Get the M13 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M13 element.
 */
static inline float caerMatrix4x4EventGetM13(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[1][3]));
}

/**
 * Set the M13 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x m13 value.
 */
static inline void caerMatrix4x4EventSetM13(caerMatrix4x4Event event, float x) {
	event->m[1][3] = htoleflt(x);
}

/**
 * Get the M20 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M20 element.
 */
static inline float caerMatrix4x4EventGetM20(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[2][0]));
}

/**
 * Set the M20 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M20 value.
 */
static inline void caerMatrix4x4EventSetM20(caerMatrix4x4Event event, float x) {
	event->m[2][0] = htoleflt(x);
}

/**
 * Get the M21 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M21 element.
 */
static inline float caerMatrix4x4EventGetM21(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[2][1]));
}

/**
 * Set the M21 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M21 value.
 */
static inline void caerMatrix4x4EventSetM21(caerMatrix4x4Event event, float x) {
	event->m[2][1] = htoleflt(x);
}

/**
 * Get the M22 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M22 element.
 */
static inline float caerMatrix4x4EventGetM22(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[2][2]));
}

/**
 * Set the M22 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M22 value.
 */
static inline void caerMatrix4x4EventSetM22(caerMatrix4x4Event event, float x) {
	event->m[2][2] = htoleflt(x);
}

/**
 * Get the M23 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M23 element.
 */
static inline float caerMatrix4x4EventGetM23(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[2][3]));
}

/**
 * Set the M23 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M23 value.
 */
static inline void caerMatrix4x4EventSetM23(caerMatrix4x4Event event, float x) {
	event->m[2][3] = htoleflt(x);
}

/**
 * Get the M30 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M30 element.
 */
static inline float caerMatrix4x4EventGetM30(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[3][0]));
}

/**
 * Set the M30 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M30 value.
 */
static inline void caerMatrix4x4EventSetM30(caerMatrix4x4Event event, float x) {
	event->m[3][0] = htoleflt(x);
}

/**
 * Get the M31 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M31 element.
 */
static inline float caerMatrix4x4EventGetM31(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[3][1]));
}

/**
 * Set the M31 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M31 value.
 */
static inline void caerMatrix4x4EventSetM31(caerMatrix4x4Event event, float x) {
	event->m[3][1] = htoleflt(x);
}

/**
 * Get the M32 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M32 element.
 */
static inline float caerMatrix4x4EventGetM32(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[3][2]));
}

/**
 * Set the M32 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M32 value.
 */
static inline void caerMatrix4x4EventSetM32(caerMatrix4x4Event event, float x) {
	event->m[3][2] = htoleflt(x);
}

/**
 * Get the M33 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 *
 * @return M33 element.
 */
static inline float caerMatrix4x4EventGetM33(caerMatrix4x4EventConst event) {
	return (leflttoh(event->m[3][3]));
}

/**
 * Set the M33 element.
 *
 * @param event a valid Matrix4x4Event pointer. Cannot be NULL.
 * @param x M33 value.
 */
static inline void caerMatrix4x4EventSetM33(caerMatrix4x4Event event, float x) {
	event->m[3][3] = htoleflt(x);
}

/**
 * Iterator over all Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4Event.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_ITERATOR_ALL_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = 0; \
		caerMatrix4x4IteratorCounter < caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader); \
		caerMatrix4x4IteratorCounter++) { \
		caerMatrix4x4Event caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEvent(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter);

/**
 * Const-Iterator over all Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4EventConst.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_CONST_ITERATOR_ALL_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = 0; \
		caerMatrix4x4IteratorCounter < caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader); \
		caerMatrix4x4IteratorCounter++) { \
		caerMatrix4x4EventConst caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEventConst(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter);

/**
 * Iterator close statement.
 */
#define CAER_MATRIX4x4_ITERATOR_ALL_END }

/**
 * Iterator over only the valid Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4Event.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_ITERATOR_VALID_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = 0; \
		caerMatrix4x4IteratorCounter < caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader); \
		caerMatrix4x4IteratorCounter++) { \
		caerMatrix4x4Event caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEvent(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter); \
		if (!caerMatrix4x4EventIsValid(caerMatrix4x4IteratorElement)) { continue; } // Skip invalid Matrix4x4 events.

/**
 * Const-Iterator over only the valid Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4EventConst.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_CONST_ITERATOR_VALID_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = 0; \
		caerMatrix4x4IteratorCounter < caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader); \
		caerMatrix4x4IteratorCounter++) { \
		caerMatrix4x4EventConst caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEventConst(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter); \
		if (!caerMatrix4x4EventIsValid(caerMatrix4x4IteratorElement)) { continue; } // Skip invalid Matrix4x4 events.

/**
 * Iterator close statement.
 */
#define CAER_MATRIX4x4_ITERATOR_VALID_END }

/**
 * Reverse iterator over all Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4Event.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_REVERSE_ITERATOR_ALL_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader) - 1; \
		caerMatrix4x4IteratorCounter >= 0; \
		caerMatrix4x4IteratorCounter--) { \
		caerMatrix4x4Event caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEvent(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter);
/**
 * Const-Reverse iterator over all Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4EventConst.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_CONST_REVERSE_ITERATOR_ALL_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader) - 1; \
		caerMatrix4x4IteratorCounter >= 0; \
		caerMatrix4x4IteratorCounter--) { \
		caerMatrix4x4EventConst caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEventConst(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter);

/**
 * Reverse iterator close statement.
 */
#define CAER_MATRIX4x4_REVERSE_ITERATOR_ALL_END }

/**
 * Reverse iterator over only the valid Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4Event.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_REVERSE_ITERATOR_VALID_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader) - 1; \
		caerMatrix4x4IteratorCounter >= 0; \
		caerMatrix4x4IteratorCounter--) { \
		caerMatrix4x4Event caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEvent(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter); \
		if (!caerMatrix4x4EventIsValid(caerMatrix4x4IteratorElement)) { continue; } // Skip invalid Matrix4x4 events.

/**
 * Const-Reverse iterator over only the valid Matrix4x4 events in a packet.
 * Returns the current index in the 'caerMatrix4x4IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerMatrix4x4IteratorElement' variable
 * of type caerMatrix4x4EventConst.
 *
 * MATRIX4x4_PACKET: a valid Matrix4x4EventPacket pointer. Cannot be NULL.
 */
#define CAER_MATRIX4x4_CONST_REVERSE_ITERATOR_VALID_START(MATRIX4x4_PACKET) \
	for (int32_t caerMatrix4x4IteratorCounter = caerEventPacketHeaderGetEventNumber(&(MATRIX4x4_PACKET)->packetHeader) - 1; \
		caerMatrix4x4IteratorCounter >= 0; \
		caerMatrix4x4IteratorCounter--) { \
		caerMatrix4x4EventConst caerMatrix4x4IteratorElement = caerMatrix4x4EventPacketGetEventConst(MATRIX4x4_PACKET, caerMatrix4x4IteratorCounter); \
		if (!caerMatrix4x4EventIsValid(caerMatrix4x4IteratorElement)) { continue; } // Skip invalid Matrix4x4 events.

/**
 * Reverse iterator close statement.
 */
#define CAER_MATRIX4x4_REVERSE_ITERATOR_VALID_END }

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_MATRIX4x4_H_ */
