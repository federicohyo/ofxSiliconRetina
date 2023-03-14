#ifndef LIBCAER_EVENTS_PACKETCONTAINER_HPP_
#define LIBCAER_EVENTS_PACKETCONTAINER_HPP_

#include <libcaer/events/packetContainer.h>
#include "common.hpp"
#include "utils.hpp"
#include <memory>
#include <utility>
#include <vector>

namespace libcaer {
namespace events {

template<class InteralIterator, class SharedPtrType> class EventPacketContainerCopyIterator {
private:
	// Original vector iterator or const_iterator.
	InteralIterator eventPacketsIterator;

	// currElement acts as a kind of cache: not only does it allow us
	// to add deep-constness (when needed), but it also stores a copy of
	// the shared_ptr we're iterating over, effectively increasing its
	// reference count by one while it is in use by the iterator and its
	// user, thus ensuring the object can never disappear from under us.
	mutable SharedPtrType currElement;

public:
	// Iterator traits.
	using iterator_category = typename InteralIterator::iterator_category;
	using value_type        = SharedPtrType;
	using pointer           = const SharedPtrType *;
	using reference         = const SharedPtrType &;
	using difference_type   = typename InteralIterator::difference_type;
	using size_type         = typename InteralIterator::difference_type;

	// Constructors.
	EventPacketContainerCopyIterator() {
		// Empty constructor fine here, results in calls to default
		// constructors for members:
		// - eventPacketsIterator() => empty/nothing iterator
		// - currElement() => empty/nullptr shared_ptr
	}

	EventPacketContainerCopyIterator(InteralIterator _eventPacketsIterator)
		: eventPacketsIterator(_eventPacketsIterator) {
		// Don't initialize currElement, it is initialized/updated
		// right before every use.
	}

	// Data access operators.
	reference operator*() const noexcept {
		currElement = *eventPacketsIterator;
		return (currElement);
	}

	pointer operator->() const noexcept {
		currElement = *eventPacketsIterator;
		return (&currElement);
	}

	reference operator[](size_type idx) const noexcept {
		currElement = eventPacketsIterator[idx];
		return (currElement);
	}

	// Comparison operators.
	bool operator==(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator == rhs.eventPacketsIterator);
	}

	bool operator!=(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator != rhs.eventPacketsIterator);
	}

	bool operator<(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator < rhs.eventPacketsIterator);
	}

	bool operator>(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator > rhs.eventPacketsIterator);
	}

	bool operator<=(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator <= rhs.eventPacketsIterator);
	}

	bool operator>=(const EventPacketContainerCopyIterator &rhs) const noexcept {
		return (eventPacketsIterator >= rhs.eventPacketsIterator);
	}

	// Prefix increment.
	EventPacketContainerCopyIterator &operator++() noexcept {
		++eventPacketsIterator;
		return (*this);
	}

	// Postfix increment.
	EventPacketContainerCopyIterator operator++(int) noexcept {
		InteralIterator currIterator = eventPacketsIterator;
		++eventPacketsIterator;
		return (EventPacketContainerCopyIterator(currIterator));
	}

	// Prefix decrement.
	EventPacketContainerCopyIterator &operator--() noexcept {
		--eventPacketsIterator;
		return (*this);
	}

	// Postfix decrement.
	EventPacketContainerCopyIterator operator--(int) noexcept {
		InteralIterator currIterator = eventPacketsIterator;
		--eventPacketsIterator;
		return (EventPacketContainerCopyIterator(currIterator));
	}

	// Iter += N.
	EventPacketContainerCopyIterator &operator+=(size_type add) noexcept {
		eventPacketsIterator += add;
		return (*this);
	}

	// Iter + N.
	EventPacketContainerCopyIterator operator+(size_type add) const noexcept {
		return (EventPacketContainerCopyIterator(eventPacketsIterator + add));
	}

	// N + Iter. Must be friend as Iter is right-hand-side.
	friend EventPacketContainerCopyIterator operator+(
		size_type lhs, const EventPacketContainerCopyIterator &rhs) noexcept {
		return (EventPacketContainerCopyIterator(rhs.eventPacketsIterator + lhs));
	}

	// Iter -= N.
	EventPacketContainerCopyIterator &operator-=(size_type sub) noexcept {
		eventPacketsIterator -= sub;
		return (*this);
	}

	// Iter - N. (N - Iter doesn't make sense!)
	EventPacketContainerCopyIterator operator-(size_type sub) const noexcept {
		return (EventPacketContainerCopyIterator(eventPacketsIterator - sub));
	}

	// Iter - Iter. (Iter + Iter doesn't make sense!)
	difference_type operator-(const EventPacketContainerCopyIterator &rhs) const noexcept {
		// Distance in pointed-to-elements.
		return (eventPacketsIterator - rhs.eventPacketsIterator);
	}

	// Swap two iterators.
	void swap(EventPacketContainerCopyIterator &rhs) noexcept {
		std::swap(eventPacketsIterator, rhs.eventPacketsIterator);
		std::swap(currElement, rhs.currElement);
	}
};

class EventPacketContainer {
private:
	/// Smallest event timestamp contained in this packet container.
	int64_t lowestEventTimestamp;
	/// Largest event timestamp contained in this packet container.
	int64_t highestEventTimestamp;
	/// Number of events contained within all the packets in this container.
	int32_t eventsNumber;
	/// Number of valid events contained within all the packets in this container.
	int32_t eventsValidNumber;
	/// Vector of pointers to the actual event packets.
	std::vector<std::shared_ptr<EventPacket>> eventPackets;

public:
	// Container traits (not really STL compatible).
	using value_type       = std::shared_ptr<EventPacket>;
	using const_value_type = std::shared_ptr<const EventPacket>;
	using size_type        = int32_t;
	using difference_type  = ptrdiff_t;

	/**
	 * Construct a new EventPacketContainer.
	 */
	EventPacketContainer()
		: lowestEventTimestamp(-1), highestEventTimestamp(-1), eventsNumber(0), eventsValidNumber(0) {
	}

	/**
	 * Construct a new EventPacketContainer with enough space to
	 * store up to the given number of event packet pointers.
	 * The pointers are present and initialized to nullptr.
	 *
	 * @param eventPacketsNumber the initial number of event packet pointers
	 *                           that can be stored in this container.
	 *                           Must be equal to one or higher.
	 */
	EventPacketContainer(size_type eventPacketsNumber)
		: lowestEventTimestamp(-1), highestEventTimestamp(-1), eventsNumber(0), eventsValidNumber(0) {
		if (eventPacketsNumber <= 0) {
			throw std::invalid_argument("Negative or zero capacity not allowed on explicit construction.");
		}

		// Initialize and fill vector after having checked size value.
		eventPackets.reserve(static_cast<size_t>(eventPacketsNumber));

		for (size_type i = 0; i < eventPacketsNumber; i++) {
			eventPackets.emplace_back(); // Call empty constructor.
		}
	}

	/**
	 * Construct a new EventPacketContainer from a C-style
	 * caerEventPacketContainer. The contained packets can take over memory
	 * ownership if so requested.
	 *
	 * @param packetContainer C-style caerEventPacketContainer from which to
	 *                        initialize the new packet container.
	 * @param takeMemoryOwnership true if the container packets shall take
	 *                            over the ownership of the memory containing
	 *                            the events from the C-style packets.
	 */
	EventPacketContainer(caerEventPacketContainer packetContainer, bool takeMemoryOwnership = true) {
		if (packetContainer == nullptr) {
			throw std::runtime_error("Failed to initialize event packet container: null pointer.");
		}

		lowestEventTimestamp  = caerEventPacketContainerGetLowestEventTimestamp(packetContainer);
		highestEventTimestamp = caerEventPacketContainerGetHighestEventTimestamp(packetContainer);
		eventsNumber          = caerEventPacketContainerGetEventsNumber(packetContainer);
		eventsValidNumber     = caerEventPacketContainerGetEventsValidNumber(packetContainer);

		// Initialize and fill vector.
		int32_t eventPacketsNumber = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

		eventPackets.reserve(static_cast<size_t>(eventPacketsNumber));

		for (size_type i = 0; i < eventPacketsNumber; i++) {
			caerEventPacketHeader packet = caerEventPacketContainerGetEventPacket(packetContainer, i);

			if (packet != nullptr) {
				eventPackets.push_back(libcaer::events::utils::makeSharedFromCStruct(packet, takeMemoryOwnership));
			}
			else {
				eventPackets.emplace_back(); // Call empty constructor.
			}
		}
	}

	// The default destructor is fine here, as it will call the vector's
	// destructor, which will call all of its content's destructors; those
	// are shared_ptr, so if their count reaches zero it will then call the
	// EventPacketHeader destructor, and everything is fine.
	// Same for copy/move assignment/constructors, the defaults are fine,
	// as vector and shared_ptr take care of all the book-keeping.

	// EventPackets vector accessors.
	size_type capacity() const noexcept {
		return (static_cast<size_type>(eventPackets.capacity()));
	}

	size_type size() const noexcept {
		return (static_cast<size_type>(eventPackets.size()));
	}

	bool empty() const noexcept {
		return (eventPackets.empty());
	}

	void clear() noexcept {
		eventPackets.clear();
	}

	/**
	 * Get the pointer to the event packet stored in this container
	 * at the given index.
	 *
	 * @param index the index of the event packet to get.
	 *
	 * @return a pointer to an event packet.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	value_type getEventPacket(size_type index) {
		// Support negative indexes to go from the end of the event packet container.
		if (index < 0) {
			index = size() + index;
		}

		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		return (eventPackets[static_cast<size_t>(index)]);
	}

	value_type operator[](size_type index) {
		return (getEventPacket(index));
	}

	/**
	 * Get the pointer to the event packet stored in this container
	 * at the given index.
	 * This is a read-only event packet, do not change its contents in any way!
	 *
	 * @param index the index of the event packet to get.
	 *
	 * @return a pointer to a read-only event packet.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	const_value_type getEventPacket(size_type index) const {
		// Support negative indexes to go from the end of the event packet container.
		if (index < 0) {
			index = size() + index;
		}

		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		return (eventPackets[static_cast<size_t>(index)]);
	}

	const_value_type operator[](size_type index) const {
		return (getEventPacket(index));
	}

	/**
	 * Set the pointer to the event packet stored in this container
	 * at the given index. The index must be valid already, this does
	 * not change the container size.
	 *
	 * @param index the index of the event packet to set.
	 * @param packetHeader a pointer to an event packet. Can be a nullptr.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	void setEventPacket(size_type index, value_type packetHeader) {
		// Support negative indexes to go from the end of the event packet container.
		if (index < 0) {
			index = size() + index;
		}

		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		eventPackets[static_cast<size_t>(index)] = packetHeader;

		updateStatistics();
	}

	/**
	 * Add an event packet pointer at the end of this container.
	 * Increases container size by one.
	 *
	 * @param packetHeader a pointer to an event packet. Can be a nullptr.
	 */
	void addEventPacket(value_type packetHeader) {
		eventPackets.push_back(packetHeader);

		updateStatistics();
	}

	/**
	 * Get the lowest timestamp contained in this event packet container.
	 *
	 * @return the lowest timestamp (in µs) or -1 if not initialized.
	 */
	int64_t getLowestEventTimestamp() const noexcept {
		return (lowestEventTimestamp);
	}

	/**
	 * Get the highest timestamp contained in this event packet container.
	 *
	 * @return the highest timestamp (in µs) or -1 if not initialized.
	 */
	int64_t getHighestEventTimestamp() const noexcept {
		return (highestEventTimestamp);
	}

	/**
	 * Get the number of events contained in this event packet container.
	 *
	 * @return the number of events in this container.
	 */
	int32_t getEventsNumber() const noexcept {
		return (eventsNumber);
	}

	/**
	 * Get the number of valid events contained in this event packet container.
	 *
	 * @return the number of valid events in this container.
	 */
	int32_t getEventsValidNumber() const noexcept {
		return (eventsValidNumber);
	}

	/**
	 * Recalculates and updates all the packet-container level statistics (event
	 * counts and timestamps).
	 */
	void updateStatistics() noexcept {
		int64_t lowestTimestamp  = -1;
		int64_t highestTimestamp = -1;
		int32_t events           = 0;
		int32_t eventsValid      = 0;

		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			// If packet has no events, skip it, it contributes nothing to statistics.
			if (packet->getEventNumber() == 0) {
				continue;
			}

			// Get timestamps to update lowest/highest tracking.
			const auto firstEvent            = packet->genericGetEvent(0);
			int64_t currLowestEventTimestamp = firstEvent.getTimestamp64();

			const auto lastEvent              = packet->genericGetEvent(-1);
			int64_t currHighestEventTimestamp = lastEvent.getTimestamp64();

			// Update tracked timestamps (or initialize if needed).
			if ((lowestTimestamp == -1) || (lowestTimestamp > currLowestEventTimestamp)) {
				lowestTimestamp = currLowestEventTimestamp;
			}

			if ((highestTimestamp == -1) || (highestTimestamp < currHighestEventTimestamp)) {
				highestTimestamp = currHighestEventTimestamp;
			}

			events += packet->getEventNumber();
			eventsValid += packet->getEventValid();
		}

		lowestEventTimestamp  = lowestTimestamp;
		highestEventTimestamp = highestTimestamp;
		eventsNumber          = events;
		eventsValidNumber     = eventsValid;
	}

	/**
	 * Get the pointer to an event packet stored in this container
	 * with the given event type. This returns the first found event packet
	 * with that type ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param typeID the event type to search for.
	 *
	 * @return a pointer to an event packet with a certain type or nullptr if none found.
	 */
	value_type findEventPacketByType(int16_t typeID) {
		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	std::unique_ptr<std::vector<value_type>> findEventPacketsByType(int16_t typeID) {
		auto results = std::unique_ptr<std::vector<value_type>>(new std::vector<value_type>());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				results->push_back(packet);
			}
		}

		return (results);
	}

	/**
	 * Get the pointer to a read-only event packet stored in this container
	 * with the given event type. This returns the first found event packet
	 * with that type ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param typeID the event type to search for.
	 *
	 * @return a pointer to a read-only event packet with a certain type or nullptr if none found.
	 */
	const_value_type findEventPacketByType(int16_t typeID) const {
		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	std::unique_ptr<std::vector<const_value_type>> findEventPacketsByType(int16_t typeID) const {
		auto results = std::unique_ptr<std::vector<const_value_type>>(new std::vector<const_value_type>());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				results->push_back(packet);
			}
		}

		return (results);
	}

	/**
	 * Get the pointer to an event packet stored in this container
	 * from the given event source. This returns the first found event packet
	 * with that source ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param sourceID the event source to search for.
	 *
	 * @return a pointer to an event packet with a certain source or nullptr if none found.
	 */
	value_type findEventPacketBySource(int16_t sourceID) {
		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventSource() == sourceID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	std::unique_ptr<std::vector<value_type>> findEventPacketsBySource(int16_t sourceID) {
		auto results = std::unique_ptr<std::vector<value_type>>(new std::vector<value_type>());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventSource() == sourceID) {
				results->push_back(packet);
			}
		}

		return (results);
	}

	/**
	 * Get the pointer to a read-only event packet stored in this container
	 * from the given event source. This returns the first found event packet
	 * with that source ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param sourceID the event source to search for.
	 *
	 * @return a pointer to a read-only event packet with a certain source or nullptr if none found.
	 */
	const_value_type findEventPacketBySource(int16_t sourceID) const {
		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventSource() == sourceID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	std::unique_ptr<std::vector<const_value_type>> findEventPacketsBySource(int16_t sourceID) const {
		auto results = std::unique_ptr<std::vector<const_value_type>>(new std::vector<const_value_type>());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventSource() == sourceID) {
				results->push_back(packet);
			}
		}

		return (results);
	}

	/**
	 * Make a deep copy of this event packet container and all of its
	 * event packets and their current events.
	 * A normal copy (using copy constructor or assignment) copies all the
	 * container's internals, and correctly handles the pointers to the
	 * event packets held in it, but those will still point to the old
	 * event packets. To also copy the individual event packets and point
	 * to the new copies, use this function.
	 *
	 * @return a deep copy of this event packet container, containing all events.
	 */
	std::unique_ptr<EventPacketContainer> copyAllEvents() const {
		std::unique_ptr<EventPacketContainer> newContainer
			= std::unique_ptr<EventPacketContainer>(new EventPacketContainer());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				newContainer->addEventPacket(nullptr);
			}
			else {
				newContainer->addEventPacket(
					std::shared_ptr<EventPacket>(packet->copy(EventPacket::copyTypes::EVENTS_ONLY)));
			}
		}

		return (newContainer);
	}

	/**
	 * Make a deep copy of this event packet container, with its event packets
	 * sized down to only include the currently valid events (eventValid),
	 * and discarding everything else.
	 * A normal copy (using copy constructor or assignment) copies all the
	 * container's internals, and correctly handles the pointers to the
	 * event packets held in it, but those will still point to the old
	 * event packets. To also copy the individual event packets and point
	 * to the new copies, use this function.
	 *
	 * @return a deep copy of this event packet container, containing only valid events.
	 */
	std::unique_ptr<EventPacketContainer> copyValidEvents() const {
		std::unique_ptr<EventPacketContainer> newContainer
			= std::unique_ptr<EventPacketContainer>(new EventPacketContainer());

		for (auto &packet : *this) {
			if (packet == nullptr) {
				newContainer->addEventPacket(nullptr);
			}
			else {
				newContainer->addEventPacket(
					std::shared_ptr<EventPacket>(packet->copy(EventPacket::copyTypes::VALID_EVENTS_ONLY)));
			}
		}

		return (newContainer);
	}

	// Iterator support (the returned shared_ptr are always read-only copies, so actual modifications to
	// what is pointed to can only happen through setEventPacket() and addEventPacket()).
	using iterator = EventPacketContainerCopyIterator<std::vector<std::shared_ptr<EventPacket>>::iterator,
		std::shared_ptr<EventPacket>>;
	using const_iterator = EventPacketContainerCopyIterator<std::vector<std::shared_ptr<EventPacket>>::const_iterator,
		std::shared_ptr<const EventPacket>>;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	iterator begin() noexcept {
		return (iterator(eventPackets.begin()));
	}

	iterator end() noexcept {
		return (iterator(eventPackets.end()));
	}

	const_iterator begin() const noexcept {
		return (cbegin());
	}

	const_iterator end() const noexcept {
		return (cend());
	}

	const_iterator cbegin() const noexcept {
		return (const_iterator(eventPackets.cbegin()));
	}

	const_iterator cend() const noexcept {
		return (const_iterator(eventPackets.cend()));
	}

	reverse_iterator rbegin() noexcept {
		return (reverse_iterator(end()));
	}

	reverse_iterator rend() noexcept {
		return (reverse_iterator(begin()));
	}

	const_reverse_iterator rbegin() const noexcept {
		return (crbegin());
	}

	const_reverse_iterator rend() const noexcept {
		return (crend());
	}

	const_reverse_iterator crbegin() const noexcept {
		return (const_reverse_iterator(cend()));
	}

	const_reverse_iterator crend() const noexcept {
		return (const_reverse_iterator(cbegin()));
	}
};
}
}

#endif /* LIBCAER_EVENTS_PACKETCONTAINER_HPP_ */
