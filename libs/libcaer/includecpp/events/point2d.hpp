#ifndef LIBCAER_EVENTS_POINT2D_HPP_
#define LIBCAER_EVENTS_POINT2D_HPP_

#include <libcaer/events/point2d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct Point2DEvent : public caer_point2d_event {
	int32_t getTimestamp() const noexcept {
		return (caerPoint2DEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerPoint2DEventGetTimestamp64(
			this, reinterpret_cast<caerPoint2DEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerPoint2DEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerPoint2DEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerPoint2DEventValidate(this, reinterpret_cast<caerPoint2DEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerPoint2DEventInvalidate(this, reinterpret_cast<caerPoint2DEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerPoint2DEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerPoint2DEventSetType(this, t));
	}

	int8_t getScale() const noexcept {
		return (caerPoint2DEventGetScale(this));
	}

	void setScale(int8_t s) noexcept {
		return (caerPoint2DEventSetScale(this, s));
	}

	float getX() const noexcept {
		return (caerPoint2DEventGetX(this));
	}

	void setX(float xVal) noexcept {
		return (caerPoint2DEventSetX(this, xVal));
	}

	float getY() const noexcept {
		return (caerPoint2DEventGetY(this));
	}

	void setY(float yVal) noexcept {
		return (caerPoint2DEventSetY(this, yVal));
	}
};

static_assert(std::is_pod<Point2DEvent>::value, "Point2DEvent is not POD.");

class Point2DEventPacket : public EventPacketCommon<Point2DEventPacket, Point2DEvent> {
public:
	// Constructors.
	Point2DEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerPoint2DEventPacket packet = caerPoint2DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	Point2DEventPacket(caerPoint2DEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, POINT2D_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	Point2DEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, POINT2D_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerPoint2DEvent evtBase
			= caerPoint2DEventPacketGetEvent(reinterpret_cast<caerPoint2DEventPacket>(header), index);
		Point2DEvent *evt = static_cast<Point2DEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerPoint2DEventConst evtBase
			= caerPoint2DEventPacketGetEventConst(reinterpret_cast<caerPoint2DEventPacketConst>(header), index);
		const Point2DEvent *evt = static_cast<const Point2DEvent *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_POINT2D_HPP_ */
