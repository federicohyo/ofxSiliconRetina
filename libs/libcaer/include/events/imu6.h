/**
 * @file imu6.h
 *
 * IMU6 (6 axes) Events format definition and handling functions.
 * This contains data coming from the Inertial Measurement Unit
 * chip, with the 3-axes accelerometer and 3-axes gyroscope.
 * Temperature is also included.
 */

#ifndef LIBCAER_EVENTS_IMU6_H_
#define LIBCAER_EVENTS_IMU6_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IMU 6-axes event data structure definition.
 * This contains accelerometer and gyroscope headings, plus
 * temperature.
 * The X, Y and Z axes are referred to the camera plane.
 * X increases to the right, Y going up and Z towards where
 * the lens is pointing. Rotation for the gyroscope is
 * counter-clockwise along the increasing axis, for all three axes.
 * Floats are in IEEE 754-2008 binary32 format.
 * Signed integers are used for fields that are to be interpreted
 * directly, for compatibility with languages that do not have
 * unsigned integer types, such as Java.
 */
PACKED_STRUCT(
struct caer_imu6_event {
	/// Event information. First because of valid mark.
	uint32_t info;
	/// Event timestamp.
	int32_t timestamp;
	/// Acceleration in the X axis, measured in g (9.81m/s²).
	float accel_x;
	/// Acceleration in the Y axis, measured in g (9.81m/s²).
	float accel_y;
	/// Acceleration in the Z axis, measured in g (9.81m/s²).
	float accel_z;
	/// Rotation in the X axis, measured in °/s.
	float gyro_x;
	/// Rotation in the Y axis, measured in °/s.
	float gyro_y;
	/// Rotation in the Z axis, measured in °/s.
	float gyro_z;
	/// Temperature, measured in °C.
	float temp;
});

/**
 * Type for pointer to IMU 6-axes event data structure.
 */
typedef struct caer_imu6_event *caerIMU6Event;
typedef const struct caer_imu6_event *caerIMU6EventConst;

/**
 * IMU 6-axes event packet data structure definition.
 * EventPackets are always made up of the common packet header,
 * followed by 'eventCapacity' events. Everything has to
 * be in one contiguous memory block.
 */
PACKED_STRUCT(
struct caer_imu6_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct caer_imu6_event events[];
});

/**
 * Type for pointer to IMU 6-axes event packet data structure.
 */
typedef struct caer_imu6_event_packet *caerIMU6EventPacket;
typedef const struct caer_imu6_event_packet *caerIMU6EventPacketConst;

/**
 * Allocate a new IMU 6-axes events packet.
 * Use free() to reclaim this memory.
 *
 * @param eventCapacity the maximum number of events this packet will hold.
 * @param eventSource the unique ID representing the source/generator of this packet.
 * @param tsOverflow the current timestamp overflow counter value for this packet.
 *
 * @return a valid IMU6EventPacket handle or NULL on error.
 */
caerIMU6EventPacket caerIMU6EventPacketAllocate(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow);

/**
 * Transform a generic event packet header into an IMU 6-axes event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid event packet header pointer. Cannot be NULL.
 * @return a properly converted, typed event packet pointer.
 */
static inline caerIMU6EventPacket caerIMU6EventPacketFromPacketHeader(caerEventPacketHeader header) {
	if (caerEventPacketHeaderGetEventType(header) != IMU6_EVENT) {
		return (NULL);
	}

	return ((caerIMU6EventPacket) header);
}

/**
 * Transform a generic read-only event packet header into a read-only IMU 6-axes event packet.
 * This takes care of proper casting and checks that the packet type really matches
 * the intended conversion type.
 *
 * @param header a valid read-only event packet header pointer. Cannot be NULL.
 * @return a properly converted, read-only typed event packet pointer.
 */
static inline caerIMU6EventPacketConst caerIMU6EventPacketFromPacketHeaderConst(caerEventPacketHeaderConst header) {
	if (caerEventPacketHeaderGetEventType(header) != IMU6_EVENT) {
		return (NULL);
	}

	return ((caerIMU6EventPacketConst) header);
}

/**
 * Get the IMU 6-axes event at the given index from the event packet.
 *
 * @param packet a valid IMU6EventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested IMU 6-axes event. NULL on error.
 */
static inline caerIMU6Event caerIMU6EventPacketGetEvent(caerIMU6EventPacket packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "IMU6 Event",
			"Called caerIMU6EventPacketGetEvent() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Get the IMU 6-axes event at the given index from the event packet.
 * This is a read-only event, do not change its contents in any way!
 *
 * @param packet a valid IMU6EventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested read-only IMU 6-axes event. NULL on error.
 */
static inline caerIMU6EventConst caerIMU6EventPacketGetEventConst(caerIMU6EventPacketConst packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "IMU6 Event",
			"Called caerIMU6EventPacketGetEventConst() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ".",
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
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return this event's 32bit microsecond timestamp.
 */
static inline int32_t caerIMU6EventGetTimestamp(caerIMU6EventConst event) {
	return (le32toh(event->timestamp));
}

/**
 * Get the 64bit event timestamp, in microseconds.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param packet the IMU6EventPacket pointer for the packet containing this event. Cannot be NULL.
 *
 * @return this event's 64bit microsecond timestamp.
 */
static inline int64_t caerIMU6EventGetTimestamp64(caerIMU6EventConst event, caerIMU6EventPacketConst packet) {
	return (I64T(
		(U64T(caerEventPacketHeaderGetEventTSOverflow(&packet->packetHeader)) << TS_OVERFLOW_SHIFT) | U64T(caerIMU6EventGetTimestamp(event))));
}

/**
 * Set the 32bit event timestamp, the value has to be in microseconds.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param timestamp a positive 32bit microsecond timestamp.
 */
static inline void caerIMU6EventSetTimestamp(caerIMU6Event event, int32_t timestamp) {
	if (timestamp < 0) {
		// Negative means using the 31st bit!
		caerLog(CAER_LOG_CRITICAL, "IMU6 Event", "Called caerIMU6EventSetTimestamp() with negative value!");
		return;
	}

	event->timestamp = htole32(timestamp);
}

/**
 * Check if this IMU 6-axes event is valid.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return true if valid, false if not.
 */
static inline bool caerIMU6EventIsValid(caerIMU6EventConst event) {
	return (GET_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK));
}

/**
 * Validate the current event by setting its valid bit to true
 * and increasing the event packet's event count and valid
 * event count. Only works on events that are invalid.
 * DO NOT CALL THIS AFTER HAVING PREVIOUSLY ALREADY
 * INVALIDATED THIS EVENT, the total count will be incorrect.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param packet the IMU6EventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerIMU6EventValidate(caerIMU6Event event, caerIMU6EventPacket packet) {
	if (!caerIMU6EventIsValid(event)) {
		SET_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK, 1);

		// Also increase number of events and valid events.
		// Only call this on (still) invalid events!
		caerEventPacketHeaderSetEventNumber(&packet->packetHeader,
			caerEventPacketHeaderGetEventNumber(&packet->packetHeader) + 1);
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) + 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "IMU6 Event", "Called caerIMU6EventValidate() on already valid event.");
	}
}

/**
 * Invalidate the current event by setting its valid bit
 * to false and decreasing the number of valid events held
 * in the packet. Only works with events that are already
 * valid!
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param packet the IMU6EventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerIMU6EventInvalidate(caerIMU6Event event, caerIMU6EventPacket packet) {
	if (caerIMU6EventIsValid(event)) {
		CLEAR_NUMBITS32(event->info, VALID_MARK_SHIFT, VALID_MARK_MASK);

		// Also decrease number of valid events. Number of total events doesn't change.
		// Only call this on valid events!
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) - 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "IMU6 Event", "Called caerIMU6EventInvalidate() on already invalid event.");
	}
}

/**
 * Get the X axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return acceleration on the X axis.
 */
static inline float caerIMU6EventGetAccelX(caerIMU6EventConst event) {
	return (leflttoh(event->accel_x));
}

/**
 * Set the X axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param accelX acceleration on the X axis.
 */
static inline void caerIMU6EventSetAccelX(caerIMU6Event event, float accelX) {
	event->accel_x = htoleflt(accelX);
}

/**
 * Get the Y axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return acceleration on the Y axis.
 */
static inline float caerIMU6EventGetAccelY(caerIMU6EventConst event) {
	return (leflttoh(event->accel_y));
}

/**
 * Set the Y axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param accelY acceleration on the Y axis.
 */
static inline void caerIMU6EventSetAccelY(caerIMU6Event event, float accelY) {
	event->accel_y = htoleflt(accelY);
}

/**
 * Get the Z axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return acceleration on the Z axis.
 */
static inline float caerIMU6EventGetAccelZ(caerIMU6EventConst event) {
	return (leflttoh(event->accel_z));
}

/**
 * Set the Z axis acceleration reading (from accelerometer).
 * This is in g (1 g = 9.81 m/s²).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param accelZ acceleration on the Z axis.
 */
static inline void caerIMU6EventSetAccelZ(caerIMU6Event event, float accelZ) {
	event->accel_z = htoleflt(accelZ);
}

/**
 * Get the X axis (roll) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return angular velocity on the X axis (roll).
 */
static inline float caerIMU6EventGetGyroX(caerIMU6EventConst event) {
	return (leflttoh(event->gyro_x));
}

/**
 * Set the X axis (roll) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param gyroX angular velocity on the X axis (roll).
 */
static inline void caerIMU6EventSetGyroX(caerIMU6Event event, float gyroX) {
	event->gyro_x = htoleflt(gyroX);
}

/**
 * Get the Y axis (pitch) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return angular velocity on the Y axis (pitch).
 */
static inline float caerIMU6EventGetGyroY(caerIMU6EventConst event) {
	return (leflttoh(event->gyro_y));
}

/**
 * Set the Y axis (pitch) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param gyroY angular velocity on the Y axis (pitch).
 */
static inline void caerIMU6EventSetGyroY(caerIMU6Event event, float gyroY) {
	event->gyro_y = htoleflt(gyroY);
}

/**
 * Get the Z axis (yaw) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return angular velocity on the Z axis (yaw).
 */
static inline float caerIMU6EventGetGyroZ(caerIMU6EventConst event) {
	return (leflttoh(event->gyro_z));
}

/**
 * Set the Z axis (yaw) angular velocity reading (from gyroscope).
 * This is in °/s (deg/sec).
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param gyroZ angular velocity on the Z axis (yaw).
 */
static inline void caerIMU6EventSetGyroZ(caerIMU6Event event, float gyroZ) {
	event->gyro_z = htoleflt(gyroZ);
}

/**
 * Get the temperature reading.
 * This is in °C.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 *
 * @return temperature in °C.
 */
static inline float caerIMU6EventGetTemp(caerIMU6EventConst event) {
	return (leflttoh(event->temp));
}

/**
 * Set the temperature reading.
 * This is in °C.
 *
 * @param event a valid IMU6Event pointer. Cannot be NULL.
 * @param temp temperature in °C.
 */
static inline void caerIMU6EventSetTemp(caerIMU6Event event, float temp) {
	event->temp = htoleflt(temp);
}

/**
 * Iterator over all IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6Event.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_ITERATOR_ALL_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = 0; \
		caerIMU6IteratorCounter < caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader); \
		caerIMU6IteratorCounter++) { \
		caerIMU6Event caerIMU6IteratorElement = caerIMU6EventPacketGetEvent(IMU6_PACKET, caerIMU6IteratorCounter);

/**
 * Const-Iterator over all IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6EventConst.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_CONST_ITERATOR_ALL_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = 0; \
		caerIMU6IteratorCounter < caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader); \
		caerIMU6IteratorCounter++) { \
		caerIMU6EventConst caerIMU6IteratorElement = caerIMU6EventPacketGetEventConst(IMU6_PACKET, caerIMU6IteratorCounter);

/**
 * Iterator close statement.
 */
#define CAER_IMU6_ITERATOR_ALL_END }

/**
 * Iterator over only the valid IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6Event.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_ITERATOR_VALID_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = 0; \
		caerIMU6IteratorCounter < caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader); \
		caerIMU6IteratorCounter++) { \
		caerIMU6Event caerIMU6IteratorElement = caerIMU6EventPacketGetEvent(IMU6_PACKET, caerIMU6IteratorCounter); \
		if (!caerIMU6EventIsValid(caerIMU6IteratorElement)) { continue; } // Skip invalid IMU6 events.

/**
 * Const-Iterator over only the valid IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6EventConst.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_CONST_ITERATOR_VALID_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = 0; \
		caerIMU6IteratorCounter < caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader); \
		caerIMU6IteratorCounter++) { \
		caerIMU6EventConst caerIMU6IteratorElement = caerIMU6EventPacketGetEventConst(IMU6_PACKET, caerIMU6IteratorCounter); \
		if (!caerIMU6EventIsValid(caerIMU6IteratorElement)) { continue; } // Skip invalid IMU6 events.

/**
 * Iterator close statement.
 */
#define CAER_IMU6_ITERATOR_VALID_END }

/**
 * Reverse iterator over all IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6Event.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_REVERSE_ITERATOR_ALL_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader) - 1; \
		caerIMU6IteratorCounter >= 0; \
		caerIMU6IteratorCounter--) { \
		caerIMU6Event caerIMU6IteratorElement = caerIMU6EventPacketGetEvent(IMU6_PACKET, caerIMU6IteratorCounter);
/**
 * Const-Reverse iterator over all IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6EventConst.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_CONST_REVERSE_ITERATOR_ALL_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader) - 1; \
		caerIMU6IteratorCounter >= 0; \
		caerIMU6IteratorCounter--) { \
		caerIMU6EventConst caerIMU6IteratorElement = caerIMU6EventPacketGetEventConst(IMU6_PACKET, caerIMU6IteratorCounter);

/**
 * Reverse iterator close statement.
 */
#define CAER_IMU6_REVERSE_ITERATOR_ALL_END }

/**
 * Reverse iterator over only the valid IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6Event.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_REVERSE_ITERATOR_VALID_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader) - 1; \
		caerIMU6IteratorCounter >= 0; \
		caerIMU6IteratorCounter--) { \
		caerIMU6Event caerIMU6IteratorElement = caerIMU6EventPacketGetEvent(IMU6_PACKET, caerIMU6IteratorCounter); \
		if (!caerIMU6EventIsValid(caerIMU6IteratorElement)) { continue; } // Skip invalid IMU6 events.

/**
 * Const-Reverse iterator over only the valid IMU6 events in a packet.
 * Returns the current index in the 'caerIMU6IteratorCounter' variable of type
 * 'int32_t' and the current read-only event in the 'caerIMU6IteratorElement' variable
 * of type caerIMU6EventConst.
 *
 * IMU6_PACKET: a valid IMU6EventPacket pointer. Cannot be NULL.
 */
#define CAER_IMU6_CONST_REVERSE_ITERATOR_VALID_START(IMU6_PACKET) \
	for (int32_t caerIMU6IteratorCounter = caerEventPacketHeaderGetEventNumber(&(IMU6_PACKET)->packetHeader) - 1; \
		caerIMU6IteratorCounter >= 0; \
		caerIMU6IteratorCounter--) { \
		caerIMU6EventConst caerIMU6IteratorElement = caerIMU6EventPacketGetEventConst(IMU6_PACKET, caerIMU6IteratorCounter); \
		if (!caerIMU6EventIsValid(caerIMU6IteratorElement)) { continue; } // Skip invalid IMU6 events.

/**
 * Reverse iterator close statement.
 */
#define CAER_IMU6_REVERSE_ITERATOR_VALID_END }

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_IMU6_H_ */
