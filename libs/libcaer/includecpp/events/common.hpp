#ifndef LIBCAER_EVENTS_COMMON_HPP_
#define LIBCAER_EVENTS_COMMON_HPP_

#include "../libcaer.hpp"

#include <libcaer/events/common.h>

#include <cassert>
#include <memory>
#include <utility>

namespace libcaer {
namespace events {

template<class T> class EventPacketIterator {
private:
	// Select proper pointer type (const or not) depending on template type.
	using eventPtrType = typename std::conditional<std::is_const<T>::value, const uint8_t *, uint8_t *>::type;

	eventPtrType eventPtr;
	size_t eventSize;

public:
	// Iterator traits.
	using iterator_category = std::random_access_iterator_tag;
	using value_type        = typename std::remove_cv<T>::type;
	using pointer           = T *;
	using reference         = T &;
	using difference_type   = ptrdiff_t;
	using size_type         = int32_t;

	// Constructors.
	EventPacketIterator() : eventPtr(nullptr), eventSize(0) {
	}

	EventPacketIterator(eventPtrType _eventPtr, size_t _eventSize) : eventPtr(_eventPtr), eventSize(_eventSize) {
	}

	// Data access operators.
	reference operator*() const noexcept {
		return (*reinterpret_cast<pointer>(eventPtr));
	}

	pointer operator->() const noexcept {
		return (reinterpret_cast<pointer>(eventPtr));
	}

	reference operator[](size_type index) const noexcept {
		return (*reinterpret_cast<pointer>(eventPtr + (index * eventSize)));
	}

	// Comparison operators.
	bool operator==(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr == rhs.eventPtr);
	}

	bool operator!=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr != rhs.eventPtr);
	}

	bool operator<(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr < rhs.eventPtr);
	}

	bool operator>(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr > rhs.eventPtr);
	}

	bool operator<=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr <= rhs.eventPtr);
	}

	bool operator>=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr >= rhs.eventPtr);
	}

	// Prefix increment.
	EventPacketIterator &operator++() noexcept {
		eventPtr += eventSize;
		return (*this);
	}

	// Postfix increment.
	EventPacketIterator operator++(int) noexcept {
		eventPtrType currPtr = eventPtr;
		eventPtr += eventSize;
		return (EventPacketIterator(currPtr, eventSize));
	}

	// Prefix decrement.
	EventPacketIterator &operator--() noexcept {
		eventPtr -= eventSize;
		return (*this);
	}

	// Postfix decrement.
	EventPacketIterator operator--(int) noexcept {
		eventPtrType currPtr = eventPtr;
		eventPtr -= eventSize;
		return (EventPacketIterator(currPtr, eventSize));
	}

	// Iter += N.
	EventPacketIterator &operator+=(size_type add) noexcept {
		eventPtr += (eventSize * add);
		return (*this);
	}

	// Iter + N.
	EventPacketIterator operator+(size_type add) const noexcept {
		return (EventPacketIterator(eventPtr + (eventSize * add), eventSize));
	}

	// N + Iter. Must be friend as Iter is right-hand-side.
	friend EventPacketIterator operator+(size_type lhs, const EventPacketIterator &rhs) noexcept {
		return (EventPacketIterator(rhs.eventPtr + (rhs.eventSize * lhs), rhs.eventSize));
	}

	// Iter -= N.
	EventPacketIterator &operator-=(size_type sub) noexcept {
		eventPtr -= (eventSize * sub);
		return (*this);
	}

	// Iter - N. (N - Iter doesn't make sense!)
	EventPacketIterator operator-(size_type sub) const noexcept {
		return (EventPacketIterator(eventPtr - (eventSize * sub), eventSize));
	}

	// Iter - Iter. (Iter + Iter doesn't make sense!)
	difference_type operator-(const EventPacketIterator &rhs) const noexcept {
		// Distance in pointed-to-elements, so of eventSize size, hence
		// why the division by eventSize is necessary.
		return ((eventPtr - rhs.eventPtr) / eventSize);
	}

	// Swap two iterators.
	void swap(EventPacketIterator &rhs) noexcept {
		std::swap(eventPtr, rhs.eventPtr);
		std::swap(eventSize, rhs.eventSize);
	}
};

class EventPacket {
protected:
	caerEventPacketHeader header;
	bool isMemoryOwner;

	// Constructors.
	EventPacket() : header(nullptr), isMemoryOwner(true) {
	}

public:
	EventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		if (caerEventPacketHeaderGetEventType(packetHeader) < CAER_DEFAULT_EVENT_TYPES_COUNT) {
			throw std::runtime_error("Failed to initialize EventPacketHeader from existing C packet header: default "
									 "event types are not allowed. "
									 "Always call the proper specialized <Type>EventPacket constructor, to guarantee "
									 "proper RTTI initialization.");
		}

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	// Destructor.
	virtual ~EventPacket() {
		// Support not freeing memory, when this packet doesn't own the memory.
		if (isMemoryOwner) {
			// All EventPackets must have been allocated somewhere on the heap,
			// and can thus always be passed to free(). free(nullptr) does nothing.
			free(header);
		}
	}

	// Copy constructor.
	EventPacket(const EventPacket &rhs) {
		// Full copy.
		header        = internalCopy(rhs.header, copyTypes::FULL);
		isMemoryOwner = true; // Always memory owner on copy!
	}

	// Copy assignment.
	EventPacket &operator=(const EventPacket &rhs) {
		// If both the same, do nothing.
		if (this != &rhs) {
			// Different packets, so we need to check if they are the same type.
			if (getEventType() != rhs.getEventType()) {
				throw std::invalid_argument("Event type must be the same.");
			}

			// They are, so we can make a copy, and if successful, put it in place
			// of the old data. internalCopy() checks for nullptr.
			caerEventPacketHeader copy = internalCopy(rhs.header, copyTypes::FULL);

			// Destroy current data, only if actually owned.
			if (isMemoryOwner) {
				free(header);
			}

			header        = copy;
			isMemoryOwner = true; // Always memory owner on copy!
		}

		return (*this);
	}

	// Move constructor.
	EventPacket(EventPacket &&rhs) noexcept {
		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Move data here.
		header        = rhs.header;
		isMemoryOwner = rhs.isMemoryOwner; // Move memory ownership too!

		// Reset old data (ready for destruction).
		rhs.header = nullptr;
	}

	// Move assignment.
	EventPacket &operator=(EventPacket &&rhs) {
		assert(this != &rhs);

		// Different packets, so we need to check if they are the same type.
		if (getEventType() != rhs.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}

		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Destroy current data, only if actually owned.
		if (isMemoryOwner) {
			free(header);
		}

		// Move data here.
		header        = rhs.header;
		isMemoryOwner = rhs.isMemoryOwner; // Move memory ownership too!

		// Reset old data (ready for destruction).
		rhs.header = nullptr;

		return (*this);
	}

	// Comparison operators.
	bool operator==(const EventPacket &rhs) const noexcept {
		return (caerEventPacketEquals(header, rhs.header));
	}

	bool operator!=(const EventPacket &rhs) const noexcept {
		return (!caerEventPacketEquals(header, rhs.header));
	}

	// Header data methods.
	int16_t getEventType() const noexcept {
		return (caerEventPacketHeaderGetEventType(header));
	}

	void setEventType(int16_t eventType) {
		if (eventType < 0) {
			throw std::invalid_argument("Negative event type not allowed.");
		}

		return (caerEventPacketHeaderSetEventType(header, eventType));
	}

	int16_t getEventSource() const noexcept {
		return (caerEventPacketHeaderGetEventSource(header));
	}

	void setEventSource(int16_t eventSource) {
		if (eventSource < 0) {
			throw std::invalid_argument("Negative event source not allowed.");
		}

		return (caerEventPacketHeaderSetEventSource(header, eventSource));
	}

	int32_t getEventSize() const noexcept {
		return (caerEventPacketHeaderGetEventSize(header));
	}

	void setEventSize(int32_t eventSize) {
		if (eventSize < 0) {
			throw std::invalid_argument("Negative event size not allowed.");
		}

		return (caerEventPacketHeaderSetEventSize(header, eventSize));
	}

	int32_t getEventTSOffset() const noexcept {
		return (caerEventPacketHeaderGetEventTSOffset(header));
	}

	void setEventTSOffset(int32_t eventTSOffset) {
		if (eventTSOffset < 0) {
			throw std::invalid_argument("Negative event TS offset not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOffset(header, eventTSOffset));
	}

	int32_t getEventTSOverflow() const noexcept {
		return (caerEventPacketHeaderGetEventTSOverflow(header));
	}

	void setEventTSOverflow(int32_t eventTSOverflow) {
		if (eventTSOverflow < 0) {
			throw std::invalid_argument("Negative event TS overflow not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOverflow(header, eventTSOverflow));
	}

	int32_t getEventCapacity() const noexcept {
		return (caerEventPacketHeaderGetEventCapacity(header));
	}

	void setEventCapacity(int32_t eventCapacity) {
		if (eventCapacity < 0) {
			throw std::invalid_argument("Negative event capacity not allowed.");
		}

		return (caerEventPacketHeaderSetEventCapacity(header, eventCapacity));
	}

	int32_t getEventNumber() const noexcept {
		return (caerEventPacketHeaderGetEventNumber(header));
	}

	void setEventNumber(int32_t eventNumber) {
		if (eventNumber < 0) {
			throw std::invalid_argument("Negative event number not allowed.");
		}

		return (caerEventPacketHeaderSetEventNumber(header, eventNumber));
	}

	int32_t getEventValid() const noexcept {
		return (caerEventPacketHeaderGetEventValid(header));
	}

	void setEventValid(int32_t eventValid) {
		if (eventValid < 0) {
			throw std::invalid_argument("Negative event valid not allowed.");
		}

		return (caerEventPacketHeaderSetEventValid(header, eventValid));
	}

	// Generic Event definiton.
	struct GenericEvent {
		const void *event;
		caerEventPacketHeaderConst header;

		int32_t getTimestamp() const noexcept {
			return (caerGenericEventGetTimestamp(event, header));
		}

		int64_t getTimestamp64() const noexcept {
			return (caerGenericEventGetTimestamp64(event, header));
		}

		bool isValid() const noexcept {
			return (caerGenericEventIsValid(event));
		}

		// C variant.
		void copy(void *eventPtrDestination, caerEventPacketHeaderConst headerPtrDestination) const {
			if (caerEventPacketHeaderGetEventType(headerPtrDestination) != caerEventPacketHeaderGetEventType(header)) {
				throw std::invalid_argument("Event type must be the same.");
			}
			if (caerEventPacketHeaderGetEventSize(headerPtrDestination) != caerEventPacketHeaderGetEventSize(header)) {
				throw std::invalid_argument("Event size must be the same.");
			}
			if (caerEventPacketHeaderGetEventTSOverflow(headerPtrDestination)
				!= caerEventPacketHeaderGetEventTSOverflow(header)) {
				throw std::invalid_argument("Event TS overflow must be the same.");
			}

			caerGenericEventCopy(eventPtrDestination, event, headerPtrDestination, header);
		}

		// C++ variants.
		void copy(struct GenericEvent &destinationEvent) const {
			copy(const_cast<void *>(destinationEvent.event), destinationEvent.header);
		}

		void copy(struct GenericEvent *destinationEvent) const {
			copy(const_cast<void *>(destinationEvent->event), destinationEvent->header);
		}
	};

	static_assert(std::is_pod<GenericEvent>::value, "GenericEvent is not POD.");

	// Container traits.
	using value_type       = GenericEvent;
	using const_value_type = const GenericEvent;
	using pointer          = GenericEvent *;
	using const_pointer    = const GenericEvent *;
	using reference        = GenericEvent &;
	using const_reference  = const GenericEvent &;
	using size_type        = int32_t;
	using difference_type  = ptrdiff_t;

	// Generic Event access methods.
	const_value_type genericGetEvent(size_type index) const {
		// Accessing elements after size() but before capacity() doesn't
		// make any sense here for Generic Events, as we only support
		// reading/querying data from those events, and that would always
		// fail for those empty events.
		const void *evt = caerGenericEventGetEvent(header, getEventIndex(index, false));

		return (GenericEvent{evt, header});
	}

	// Generic Event Packet methods.
	int64_t getDataSize() const noexcept {
		return (caerEventPacketGetDataSize(header));
	}

	int64_t getSize() const noexcept {
		return (caerEventPacketGetSize(header));
	}

	int64_t getDataSizeEvents() const noexcept {
		return (caerEventPacketGetDataSizeEvents(header));
	}

	int64_t getSizeEvents() const noexcept {
		return (caerEventPacketGetSizeEvents(header));
	}

	void clear() noexcept {
		caerEventPacketClear(header);
	}

	void clean() noexcept {
		caerEventPacketClean(header);
	}

	void resize(size_type newEventCapacity) {
		if (newEventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed.");
		}

		caerEventPacketHeader resizedPacket = caerEventPacketResize(header, newEventCapacity);
		if (resizedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = resizedPacket;
		}
	}

	void shrink_to_fit() {
		size_type shrunkSize = getEventValid();

		// Zero resize not allowed, must be at least one.
		if (shrunkSize == 0) {
			shrunkSize = 1;
		}

		resize(shrunkSize);
	}

	void grow(size_type newEventCapacity) {
		if (newEventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed.");
		}
		if (newEventCapacity <= getEventCapacity()) {
			throw std::invalid_argument("New event capacity must be strictly bigger than old one.");
		}

		caerEventPacketHeader enlargedPacket = caerEventPacketGrow(header, newEventCapacity);
		if (enlargedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = enlargedPacket;
		}
	}

	void append(const EventPacket &appendPacket) {
		if (getEventType() != appendPacket.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}
		if (getEventSize() != appendPacket.getEventSize()) {
			throw std::invalid_argument("Event size must be the same.");
		}
		if (getEventTSOverflow() != appendPacket.getEventTSOverflow()) {
			throw std::invalid_argument("Event TS overflow must be the same.");
		}

		caerEventPacketHeader mergedPacket = caerEventPacketAppend(header, appendPacket.header);
		if (mergedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = mergedPacket;
		}
	}

	enum class copyTypes { FULL, EVENTS_ONLY, VALID_EVENTS_ONLY };

	std::unique_ptr<EventPacket> copy(copyTypes ct) const {
		return (virtualCopy(ct));
	}

	// Swap two event packets.
	void swap(EventPacket &rhs) {
		if (getEventType() != rhs.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}

		std::swap(header, rhs.header);
		std::swap(isMemoryOwner, rhs.isMemoryOwner);
	}

	// Direct underlying pointer access.
	caerEventPacketHeader getHeaderPointer() noexcept {
		return (header);
	}

	caerEventPacketHeaderConst getHeaderPointer() const noexcept {
		return (header);
	}

	// Memory ownership information.
	bool isPacketMemoryOwner() const noexcept {
		return (isMemoryOwner);
	}

	// Direct underlying pointer access for conversion back to C.
	caerEventPacketHeader getHeaderPointerForCOutput() noexcept {
		isMemoryOwner = false;

		return (header);
	}

	// Convenience methods.
	size_type capacity() const noexcept {
		return (getEventCapacity());
	}

	size_type size() const noexcept {
		return (getEventNumber());
	}

	bool empty() const noexcept {
		return (getEventNumber() == 0);
	}

protected:
	// Internal copy functions.
	virtual std::unique_ptr<EventPacket> virtualCopy(copyTypes ct) const {
		return (std::unique_ptr<EventPacket>(new EventPacket(internalCopy(header, ct))));
	}

	static caerEventPacketHeader internalCopy(caerEventPacketHeaderConst header, copyTypes ct) {
		caerEventPacketHeader packetCopy = nullptr;

		switch (ct) {
			case copyTypes::FULL:
				packetCopy = caerEventPacketCopy(header);
				break;

			case copyTypes::EVENTS_ONLY:
				packetCopy = caerEventPacketCopyOnlyEvents(header);
				break;

			case copyTypes::VALID_EVENTS_ONLY:
				packetCopy = caerEventPacketCopyOnlyValidEvents(header);
				break;
		}

		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (packetCopy);
	}

	// Constructor checks.
	static void constructorCheckCapacitySourceTSOverflow(
		size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}
		if (eventSource < 0) {
			throw std::invalid_argument("Negative event source not allowed.");
		}
		if (tsOverflow < 0) {
			throw std::invalid_argument("Negative event TS overflow not allowed.");
		}
	}

	static void constructorCheckNullptr(const void *packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet: null pointer.");
		}
	}

	static void constructorCheckEventType(caerEventPacketHeaderConst packet, int16_t type) {
		if (caerEventPacketHeaderGetEventType(packet) != type) {
			throw std::runtime_error("Failed to initialize event packet: wrong type.");
		}
	}

	size_type getEventIndex(size_type index, bool limitIsCapacity) const {
		// Support negative indexes to go from the last existing/defined event
		// backwards (not from the capacity!).
		if (index < 0) {
			index = size() + index;
		}

		if (index < 0 || index >= ((limitIsCapacity) ? (capacity()) : (size()))) {
			throw std::out_of_range("Index out of range.");
		}

		return (index);
	}
};

template<class PKT, class EVT> class EventPacketCommon : public EventPacket {
public:
	// Container traits.
	using value_type       = EVT;
	using const_value_type = const EVT;
	using pointer          = EVT *;
	using const_pointer    = const EVT *;
	using reference        = EVT &;
	using const_reference  = const EVT &;
	using size_type        = int32_t;
	using difference_type  = ptrdiff_t;

	// Event access methods.
	reference getEvent(size_type index) {
		return (virtualGetEvent(getEventIndex(index, true)));
	}

	const_reference getEvent(size_type index) const {
		return (virtualGetEvent(getEventIndex(index, true)));
	}

	reference operator[](size_type index) {
		return (getEvent(index));
	}

	const_reference operator[](size_type index) const {
		return (getEvent(index));
	}

	reference front() {
		return (getEvent(0));
	}

	const_reference front() const {
		return (getEvent(0));
	}

	reference back() {
		// On empty packet, define back() as returning the same element
		// as front(), the first one, which always exists due to minimum
		// packet capacity being 1.
		if (size() == 0) {
			return (getEvent(0));
		}

		return (getEvent(-1));
	}

	const_reference back() const {
		// On empty packet, define back() as returning the same element
		// as front(), the first one, which always exists due to minimum
		// packet capacity being 1.
		if (size() == 0) {
			return (getEvent(0));
		}

		return (getEvent(-1));
	}

	std::unique_ptr<PKT> copy(copyTypes ct) const {
		return (std::unique_ptr<PKT>(static_cast<PKT *>(virtualCopy(ct).release())));
	}

	// Iterator support.
	using iterator               = EventPacketIterator<value_type>;
	using const_iterator         = EventPacketIterator<const_value_type>;
	using reverse_iterator       = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	iterator begin() noexcept {
		return (iterator(reinterpret_cast<uint8_t *>(&front()), static_cast<size_t>(getEventSize())));
	}

	iterator end() noexcept {
		// Pointer must be to element one past the end!
		return (iterator(reinterpret_cast<uint8_t *>(&back()) + getEventSize(), static_cast<size_t>(getEventSize())));
	}

	const_iterator begin() const noexcept {
		return (cbegin());
	}

	const_iterator end() const noexcept {
		return (cend());
	}

	const_iterator cbegin() const noexcept {
		return (const_iterator(reinterpret_cast<const uint8_t *>(&front()), static_cast<size_t>(getEventSize())));
	}

	const_iterator cend() const noexcept {
		// Pointer must be to element one past the end!
		return (const_iterator(
			reinterpret_cast<const uint8_t *>(&back()) + getEventSize(), static_cast<size_t>(getEventSize())));
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

protected:
	std::unique_ptr<EventPacket> virtualCopy(copyTypes ct) const override {
		return (std::unique_ptr<PKT>(new PKT(internalCopy(header, ct))));
	}

	virtual reference virtualGetEvent(size_type index) noexcept = 0;

	virtual const_reference virtualGetEvent(size_type index) const noexcept = 0;
};
} // namespace events
} // namespace libcaer

#endif /* LIBCAER_EVENTS_COMMON_HPP_ */
