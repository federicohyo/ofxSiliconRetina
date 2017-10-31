/**
 * @file packetContainer.h
 *
 * EventPacketContainer format definition and handling functions.
 * An EventPacketContainer is a logical construct that contains packets
 * of events (EventPackets) of different event types, with the aim of
 * keeping related events of differing types, such as DVS and IMU data,
 * together. Such a relation is usually based on time intervals, trying
 * to keep groups of event happening in a certain time-slice together.
 * This time-order is based on the *main* time-stamp of an event, the one
 * whose offset is referenced in the event packet header and that is
 * used by the caerGenericEvent*() functions. It's guaranteed that all
 * conforming input modules keep to this rule, generating containers
 * that include all events from all types within the given time-slice.
 * The smallest and largest timestamps are tracked at the packet container
 * level as a convenience, to avoid having to examine all packets for
 * this often useful piece of information.
 * All integers are in their native host format, as this is a purely
 * internal, in-memory data structure, never meant for exchange between
 * different systems (and different endianness).
 *
 * == Packet Containers and Input Modules ==
 * The "packeting system" works in this way: events are accumulated by
 * type in a packet, and that packet is part of a packet container, by
 * an input module.
 * The packet container is then sent out for processing when either the
 * configured time limit or the size limit are hit.
 * The time limit is always active, in microseconds, and basically tells
 * you the time-span an event packet covers. This enables regular, constant
 * delivery of packets, that cover a period of time.
 * The size limit is an addon to prevent packets to grow to immense sizes
 * (like if the time limit is high and there is lots of activity). As soon
 * as a packet hits the number of events in the size limit, it is sent out.
 * The regular time limit is not reset in this case. This size limit can
 * be disabled by setting it to 0.
 * The cAER DVS128/DAVIS/File/Network input modules call these two
 * configuration variables "PacketContainerInterval" and "PacketContainerMaxPacketSize".
 * Too small packet sizes or intervals simply mean more packets, which may
 * negatively affect performance. It's usually a good idea to set the size
 * to something around 4-8K, and the time to a good value based on the
 * application you're building, so if you need ms-reaction-time, you probably
 * want to set it to 1000µs, so that you do get new data every ms. If on the
 * other hand you're looking at a static scene and just want to detect that
 * something is passing by once every while, a higher number like 100ms
 * might also be perfectly appropriate.
 */

#ifndef LIBCAER_EVENTS_PACKETCONTAINER_H_
#define LIBCAER_EVENTS_PACKETCONTAINER_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * EventPacketContainer data structure definition.
 * Signed integers are used for compatibility with languages that
 * do not have unsigned ones, such as Java.
 */
PACKED_STRUCT(
struct caer_event_packet_container {
	/// Smallest event timestamp contained in this packet container.
	int64_t lowestEventTimestamp;
	/// Largest event timestamp contained in this packet container.
	int64_t highestEventTimestamp;
	/// Number of events contained within all the packets in this container.
	int32_t eventsNumber;
	/// Number of valid events contained within all the packets in this container.
	int32_t eventsValidNumber;
	/// Number of different event packets contained.
	int32_t eventPacketsNumber;
	/// Array of pointers to the actual event packets.
	caerEventPacketHeader eventPackets[];
});

/**
 * Type for pointer to EventPacketContainer data structure.
 */
typedef struct caer_event_packet_container *caerEventPacketContainer;
typedef const struct caer_event_packet_container *caerEventPacketContainerConst;

/**
 * Allocate a new EventPacketContainer with enough space to
 * store up to the given number of EventPacket pointers.
 * All packet pointers will be NULL initially.
 *
 * @param eventPacketsNumber the maximum number of EventPacket pointers
 *                           that can be stored in this container.
 *
 * @return a valid EventPacketContainer handle or NULL on error.
 */
caerEventPacketContainer caerEventPacketContainerAllocate(int32_t eventPacketsNumber);

/**
 * Free the memory occupied by an EventPacketContainer, as well as
 * freeing all of its contained EventPackets and their memory.
 * If you don't want the contained EventPackets to be freed, make
 * sure that you set their pointers to NULL before calling this.

 * @param container the container to be freed.
 */
void caerEventPacketContainerFree(caerEventPacketContainer container);

// Forward declaration for use in set operations.
static inline void caerEventPacketContainerUpdateStatistics(caerEventPacketContainer container);

/**
 * Get the maximum number of EventPacket pointers that can be stored
 * in this particular EventPacketContainer.
 *
 * @param container a valid EventPacketContainer handle. If NULL, zero is returned.
 *
 * @return the number of EventPacket pointers that can be contained.
 */
static inline int32_t caerEventPacketContainerGetEventPacketsNumber(caerEventPacketContainerConst container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (0);
	}

	return (container->eventPacketsNumber);
}

/**
 * Set the maximum number of EventPacket pointers that can be stored
 * in this particular EventPacketContainer. This should never be used
 * directly, caerEventPacketContainerAllocate() sets this for you.
 *
 * @param container a valid EventPacketContainer handle. If NULL, nothing happens.
 * @param eventPacketsNumber the number of EventPacket pointers that can be contained.
 */
static inline void caerEventPacketContainerSetEventPacketsNumber(caerEventPacketContainer container,
	int32_t eventPacketsNumber) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return;
	}

	if (eventPacketsNumber < 0) {
		// Negative numbers (bit 31 set) are not allowed!
		caerLog(CAER_LOG_CRITICAL, "EventPacket Container",
			"Called caerEventPacketContainerSetEventPacketsNumber() with negative value!");
		return;
	}

	container->eventPacketsNumber = eventPacketsNumber;

	// Always update all the statics on set operation.
	caerEventPacketContainerUpdateStatistics(container);
}

/**
 * Get the pointer to the EventPacket stored in this container
 * at the given index.
 *
 * @param container a valid EventPacketContainer handle. If NULL, returns NULL too.
 * @param n the index of the EventPacket to get.
 *
 * @return a pointer to an EventPacket or NULL on error.
 */
static inline caerEventPacketHeader caerEventPacketContainerGetEventPacket(caerEventPacketContainerConst container,
	int32_t n) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (NULL);
	}

	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketContainerGetEventPacketsNumber(container)) {
		caerLog(CAER_LOG_CRITICAL, "EventPacket Container",
			"Called caerEventPacketContainerGetEventPacket() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ". Negative values are not allowed!",
			n, caerEventPacketContainerGetEventPacketsNumber(container) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event packet.
	return (container->eventPackets[n]);
}

/**
 * Get the pointer to the EventPacket stored in this container
 * at the given index.
 * This is a read-only EventPacket, do not change its contents in any way!
 *
 * @param container a valid EventPacketContainer handle. If NULL, returns NULL too.
 * @param n the index of the EventPacket to get.
 *
 * @return a pointer to a read-only EventPacket or NULL on error.
 */
static inline caerEventPacketHeaderConst caerEventPacketContainerGetEventPacketConst(caerEventPacketContainerConst container, int32_t n) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (NULL);
	}

	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketContainerGetEventPacketsNumber(container)) {
		caerLog(CAER_LOG_CRITICAL, "EventPacket Container",
			"Called caerEventPacketContainerGetEventPacketConst() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ". Negative values are not allowed!",
			n, caerEventPacketContainerGetEventPacketsNumber(container) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event packet.
	return (container->eventPackets[n]);
}

/**
 * Set the pointer to the EventPacket stored in this container
 * at the given index.
 *
 * @param container a valid EventPacketContainer handle. If NULL, nothing happens.
 * @param n the index of the EventPacket to set.
 * @param packetHeader a pointer to an EventPacket's header. Can be NULL.
 */
static inline void caerEventPacketContainerSetEventPacket(caerEventPacketContainer container, int32_t n,
	caerEventPacketHeader packetHeader) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return;
	}

	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketContainerGetEventPacketsNumber(container)) {
		caerLog(CAER_LOG_CRITICAL, "EventPacket Container",
			"Called caerEventPacketContainerSetEventPacket() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ". Negative values are not allowed!",
			n, caerEventPacketContainerGetEventPacketsNumber(container) - 1);
		return;
	}

	// Store the given event packet.
	container->eventPackets[n] = packetHeader;

	// Always update all the statics on set operation.
	caerEventPacketContainerUpdateStatistics(container);
}

/**
 * Get the lowest timestamp contained in this event packet container.
 *
 * @param container a valid EventPacketContainer handle. If NULL, -1 is returned.
 *
 * @return the lowest timestamp (in µs) or -1 if not initialized.
 */
static inline int64_t caerEventPacketContainerGetLowestEventTimestamp(caerEventPacketContainerConst container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (-1);
	}

	return (container->lowestEventTimestamp);
}

/**
 * Get the highest timestamp contained in this event packet container.
 *
 * @param container a valid EventPacketContainer handle. If NULL, -1 is returned.
 *
 * @return the highest timestamp (in µs) or -1 if not initialized.
 */
static inline int64_t caerEventPacketContainerGetHighestEventTimestamp(caerEventPacketContainerConst container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (-1);
	}

	return (container->highestEventTimestamp);
}

/**
 * Get the number of events contained in this event packet container.
 *
 * @param container a valid EventPacketContainer handle. If NULL, 0 is returned.
 *
 * @return the number of events in this container.
 */
static inline int32_t caerEventPacketContainerGetEventsNumber(caerEventPacketContainerConst container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (0);
	}

	return (container->eventsNumber);
}

/**
 * Get the number of valid events contained in this event packet container.
 *
 * @param container a valid EventPacketContainer handle. If NULL, 0 is returned.
 *
 * @return the number of valid events in this container.
 */
static inline int32_t caerEventPacketContainerGetEventsValidNumber(caerEventPacketContainerConst container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (0);
	}

	return (container->eventsValidNumber);
}

/**
 * Iterator over all event packets in an event packet container.
 * Returns the current index in the 'caerEventPacketContainerIteratorCounter' variable
 * of type 'int32_t' and the current event packet in the 'caerEventPacketContainerIteratorElement'
 * variable of type caerEventPacketHeader. The current packet may be NULL, in which case it is
 * skipped during iteration.
 *
 * PACKET_CONTAINER: a valid EventPacketContainer handle. If NULL, no iteration is performed.
 */
#define CAER_EVENT_PACKET_CONTAINER_ITERATOR_START(PACKET_CONTAINER) \
	if ((PACKET_CONTAINER) != NULL) { \
		for (int32_t caerEventPacketContainerIteratorCounter = 0; \
			caerEventPacketContainerIteratorCounter < caerEventPacketContainerGetEventPacketsNumber(PACKET_CONTAINER); \
			caerEventPacketContainerIteratorCounter++) { \
			caerEventPacketHeader caerEventPacketContainerIteratorElement = caerEventPacketContainerGetEventPacket(PACKET_CONTAINER, caerEventPacketContainerIteratorCounter); \
			if (caerEventPacketContainerIteratorElement == NULL) { continue; }

/**
 * Const-Iterator over all event packets in an event packet container.
 * Returns the current index in the 'caerEventPacketContainerIteratorCounter' variable
 * of type 'int32_t' and the current read-only event packet in the 'caerEventPacketContainerIteratorElement'
 * variable of type caerEventPacketHeaderConst. The current packet may be NULL, in which case it is
 * skipped during iteration.
 *
 * PACKET_CONTAINER: a valid EventPacketContainer handle. If NULL, no iteration is performed.
 */
#define CAER_EVENT_PACKET_CONTAINER_CONST_ITERATOR_START(PACKET_CONTAINER) \
	if ((PACKET_CONTAINER) != NULL) { \
		for (int32_t caerEventPacketContainerIteratorCounter = 0; \
			caerEventPacketContainerIteratorCounter < caerEventPacketContainerGetEventPacketsNumber(PACKET_CONTAINER); \
			caerEventPacketContainerIteratorCounter++) { \
			caerEventPacketHeaderConst caerEventPacketContainerIteratorElement = caerEventPacketContainerGetEventPacketConst(PACKET_CONTAINER, caerEventPacketContainerIteratorCounter); \
			if (caerEventPacketContainerIteratorElement == NULL) { continue; }

/**
 * Iterator close statement.
 */
#define CAER_EVENT_PACKET_CONTAINER_ITERATOR_END } }

/**
 * Recalculates and updates all the packet-container level statistics (event
 * counts and timestamps).
 *
 * @param container a valid EventPacketContainer handle. If NULL, nothing happens.
 */
static inline void caerEventPacketContainerUpdateStatistics(caerEventPacketContainer container) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return;
	}

	int64_t lowestTimestamp = -1;
	int64_t highestTimestamp = -1;
	int32_t eventsNumber = 0;
	int32_t eventsValid = 0;

	CAER_EVENT_PACKET_CONTAINER_CONST_ITERATOR_START(container)
		// If packet has no events, skip it, it contributes nothing to statistics.
		if (caerEventPacketHeaderGetEventNumber(caerEventPacketContainerIteratorElement) == 0) {
			continue;
		}

		// Get timestamps to update lowest/highest tracking.
		const void *firstEvent = caerGenericEventGetEvent(caerEventPacketContainerIteratorElement, 0);
		int64_t currLowestEventTimestamp = caerGenericEventGetTimestamp64(firstEvent, caerEventPacketContainerIteratorElement);

		const void *lastEvent = caerGenericEventGetEvent(caerEventPacketContainerIteratorElement,
			caerEventPacketHeaderGetEventNumber(caerEventPacketContainerIteratorElement) - 1);
		int64_t currHighestEventTimestamp = caerGenericEventGetTimestamp64(lastEvent, caerEventPacketContainerIteratorElement);

		// Update tracked timestamps (or initialize if needed).
		if ((lowestTimestamp == -1) || (lowestTimestamp > currLowestEventTimestamp)) {
			lowestTimestamp = currLowestEventTimestamp;
		}

		if ((highestTimestamp == -1) || (highestTimestamp < currHighestEventTimestamp)) {
			highestTimestamp = currHighestEventTimestamp;
		}

		eventsNumber += caerEventPacketHeaderGetEventNumber(caerEventPacketContainerIteratorElement);
		eventsValid += caerEventPacketHeaderGetEventValid(caerEventPacketContainerIteratorElement);
	CAER_EVENT_PACKET_CONTAINER_ITERATOR_END

	container->lowestEventTimestamp = lowestTimestamp;
	container->highestEventTimestamp = highestTimestamp;
	container->eventsNumber = eventsNumber;
	container->eventsValidNumber = eventsValid;
}

/**
 * Get the pointer to an EventPacket stored in this container
 * with the given event type. This returns the first found event packet
 * with that type ID, or NULL if we get to the end without finding any
 * such event packet.
 *
 * @param container a valid EventPacketContainer handle. If NULL, returns NULL too.
 * @param typeID the event type to search for.
 *
 * @return a pointer to an EventPacket with a certain type or NULL if none found.
 */
static inline caerEventPacketHeader caerEventPacketContainerFindEventPacketByType(caerEventPacketContainerConst container,
	int16_t typeID) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (NULL);
	}

	CAER_EVENT_PACKET_CONTAINER_ITERATOR_START(container)
		if (caerEventPacketHeaderGetEventType(caerEventPacketContainerIteratorElement) == typeID) {
			// Found it, return it.
			return (caerEventPacketContainerIteratorElement);
		}
	CAER_EVENT_PACKET_CONTAINER_ITERATOR_END

	// Found nothing, return nothing.
	return (NULL);
}

/**
 * Get the pointer to a read-only EventPacket stored in this container
 * with the given event type. This returns the first found event packet
 * with that type ID, or NULL if we get to the end without finding any
 * such event packet.
 *
 * @param container a valid EventPacketContainer handle. If NULL, returns NULL too.
 * @param typeID the event type to search for.
 *
 * @return a pointer to a read-only EventPacket with a certain type or NULL if none found.
 */
static inline caerEventPacketHeaderConst caerEventPacketContainerFindEventPacketByTypeConst(caerEventPacketContainerConst container,
	int16_t typeID) {
	// Non-existing (empty) containers have no valid packets in them!
	if (container == NULL) {
		return (NULL);
	}

	CAER_EVENT_PACKET_CONTAINER_CONST_ITERATOR_START(container)
		if (caerEventPacketHeaderGetEventType(caerEventPacketContainerIteratorElement) == typeID) {
			// Found it, return it.
			return (caerEventPacketContainerIteratorElement);
		}
	CAER_EVENT_PACKET_CONTAINER_ITERATOR_END

	// Found nothing, return nothing.
	return (NULL);
}

/**
 * Make a deep copy of an event packet container and all of its
 * event packets and their current events.
 *
 * @param container an event packet container to copy.
 *
 * @return a deep copy of an event packet container, containing all events.
 */
static inline caerEventPacketContainer caerEventPacketContainerCopyAllEvents(caerEventPacketContainerConst container) {
	if (container == NULL) {
		return (NULL);
	}

	caerEventPacketContainer newContainer = caerEventPacketContainerAllocate(
		caerEventPacketContainerGetEventsNumber(container));
	if (newContainer == NULL) {
		return (NULL);
	}

	CAER_EVENT_PACKET_CONTAINER_CONST_ITERATOR_START(container)
		caerEventPacketContainerSetEventPacket(newContainer, caerEventPacketContainerIteratorCounter,
			caerEventPacketCopyOnlyEvents(caerEventPacketContainerIteratorElement));
	CAER_EVENT_PACKET_CONTAINER_ITERATOR_END

	return (newContainer);
}

/**
 * Make a deep copy of an event packet container, with its event packets
 * sized down to only include the currently valid events (eventValid),
 * and discarding everything else.
 *
 * @param container an event packet container to copy.
 *
 * @return a deep copy of an event packet container, containing only valid events.
 */
static inline caerEventPacketContainer caerEventPacketContainerCopyValidEvents(caerEventPacketContainerConst container) {
	if (container == NULL) {
		return (NULL);
	}

	caerEventPacketContainer newContainer = caerEventPacketContainerAllocate(
		caerEventPacketContainerGetEventsNumber(container));
	if (newContainer == NULL) {
		return (NULL);
	}

	CAER_EVENT_PACKET_CONTAINER_CONST_ITERATOR_START(container)
		caerEventPacketContainerSetEventPacket(newContainer, caerEventPacketContainerIteratorCounter,
			caerEventPacketCopyOnlyValidEvents(caerEventPacketContainerIteratorElement));
	CAER_EVENT_PACKET_CONTAINER_ITERATOR_END

	return (newContainer);
}

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_PACKETCONTAINER_H_ */
