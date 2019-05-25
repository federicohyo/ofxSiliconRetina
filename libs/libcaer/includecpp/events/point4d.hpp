#ifndef LIBCAER_EVENTS_POINT4D_HPP_
#define LIBCAER_EVENTS_POINT4D_HPP_

#include <libcaer/events/point4d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct Point4DEvent : public caer_point4d_event {
	int32_t getTimestamp() const noexcept {
		return (caerPoint4DEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerPoint4DEventGetTimestamp64(
			this, reinterpret_cast<caerPoint4DEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerPoint4DEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerPoint4DEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerPoint4DEventValidate(this, reinterpret_cast<caerPoint4DEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerPoint4DEventInvalidate(this, reinterpret_cast<caerPoint4DEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerPoint4DEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerPoint4DEventSetType(this, t));
	}

	int8_t getScale() const noexcept {
		return (caerPoint4DEventGetScale(this));
	}

	void setScale(int8_t s) noexcept {
		return (caerPoint4DEventSetScale(this, s));
	}

	float getX() const noexcept {
		return (caerPoint4DEventGetX(this));
	}

	void setX(float xVal) noexcept {
		return (caerPoint4DEventSetX(this, xVal));
	}

	float getY() const noexcept {
		return (caerPoint4DEventGetY(this));
	}

	void setY(float yVal) noexcept {
		return (caerPoint4DEventSetY(this, yVal));
	}

	float getZ() const noexcept {
		return (caerPoint4DEventGetZ(this));
	}

	void setZ(float zVal) noexcept {
		return (caerPoint4DEventSetZ(this, zVal));
	}

	float getW() const noexcept {
		return (caerPoint4DEventGetW(this));
	}

	void setW(float wVal) noexcept {
		return (caerPoint4DEventSetW(this, wVal));
	}
};

static_assert(std::is_pod<Point4DEvent>::value, "Point4DEvent is not POD.");

class Point4DEventPacket : public EventPacketCommon<Point4DEventPacket, Point4DEvent> {
public:
	// Constructors.
	Point4DEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerPoint4DEventPacket packet = caerPoint4DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	Point4DEventPacket(caerPoint4DEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, POINT4D_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	Point4DEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, POINT4D_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerPoint4DEvent evtBase
			= caerPoint4DEventPacketGetEvent(reinterpret_cast<caerPoint4DEventPacket>(header), index);
		Point4DEvent *evt = static_cast<Point4DEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerPoint4DEventConst evtBase
			= caerPoint4DEventPacketGetEventConst(reinterpret_cast<caerPoint4DEventPacketConst>(header), index);
		const Point4DEvent *evt = static_cast<const Point4DEvent *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_POINT4D_HPP_ */
