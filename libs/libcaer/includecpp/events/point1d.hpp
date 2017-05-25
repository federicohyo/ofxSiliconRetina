#ifndef LIBCAER_EVENTS_POINT1D_HPP_
#define LIBCAER_EVENTS_POINT1D_HPP_

#include <libcaer/events/point1d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct Point1DEvent: public caer_point1d_event {
	int32_t getTimestamp() const noexcept {
		return (caerPoint1DEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerPoint1DEventGetTimestamp64(this,
			reinterpret_cast<caerPoint1DEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerPoint1DEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerPoint1DEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerPoint1DEventValidate(this, reinterpret_cast<caerPoint1DEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerPoint1DEventInvalidate(this, reinterpret_cast<caerPoint1DEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerPoint1DEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerPoint1DEventSetType(this, t));
	}

	int8_t getScale() const noexcept {
		return (caerPoint1DEventGetScale(this));
	}

	void setScale(int8_t s) noexcept {
		return (caerPoint1DEventSetScale(this, s));
	}

	float getX() const noexcept {
		return (caerPoint1DEventGetX(this));
	}

	void setX(float xVal) noexcept {
		return (caerPoint1DEventSetX(this, xVal));
	}
};

static_assert(std::is_pod<Point1DEvent>::value, "Point1DEvent is not POD.");

class Point1DEventPacket: public EventPacketCommon<Point1DEventPacket, Point1DEvent> {
public:
	// Constructors.
	Point1DEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerPoint1DEventPacket packet = caerPoint1DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	Point1DEventPacket(caerPoint1DEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, POINT1D_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	Point1DEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, POINT1D_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerPoint1DEvent evtBase = caerPoint1DEventPacketGetEvent(reinterpret_cast<caerPoint1DEventPacket>(header),
			index);
		Point1DEvent *evt = static_cast<Point1DEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerPoint1DEventConst evtBase = caerPoint1DEventPacketGetEventConst(
			reinterpret_cast<caerPoint1DEventPacketConst>(header), index);
		const Point1DEvent *evt = static_cast<const Point1DEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_POINT1D_HPP_ */
