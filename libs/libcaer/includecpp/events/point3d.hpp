#ifndef LIBCAER_EVENTS_POINT3D_HPP_
#define LIBCAER_EVENTS_POINT3D_HPP_

#include <libcaer/events/point3d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct Point3DEvent : public caer_point3d_event {
	int32_t getTimestamp() const noexcept {
		return (caerPoint3DEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerPoint3DEventGetTimestamp64(
			this, reinterpret_cast<caerPoint3DEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerPoint3DEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerPoint3DEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerPoint3DEventValidate(this, reinterpret_cast<caerPoint3DEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerPoint3DEventInvalidate(this, reinterpret_cast<caerPoint3DEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerPoint3DEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerPoint3DEventSetType(this, t));
	}

	int8_t getScale() const noexcept {
		return (caerPoint3DEventGetScale(this));
	}

	void setScale(int8_t s) noexcept {
		return (caerPoint3DEventSetScale(this, s));
	}

	float getX() const noexcept {
		return (caerPoint3DEventGetX(this));
	}

	void setX(float xVal) noexcept {
		return (caerPoint3DEventSetX(this, xVal));
	}

	float getY() const noexcept {
		return (caerPoint3DEventGetY(this));
	}

	void setY(float yVal) noexcept {
		return (caerPoint3DEventSetY(this, yVal));
	}

	float getZ() const noexcept {
		return (caerPoint3DEventGetZ(this));
	}

	void setZ(float zVal) noexcept {
		return (caerPoint3DEventSetZ(this, zVal));
	}
};

static_assert(std::is_pod<Point3DEvent>::value, "Point3DEvent is not POD.");

class Point3DEventPacket : public EventPacketCommon<Point3DEventPacket, Point3DEvent> {
public:
	// Constructors.
	Point3DEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerPoint3DEventPacket packet = caerPoint3DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	Point3DEventPacket(caerPoint3DEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, POINT3D_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	Point3DEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, POINT3D_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerPoint3DEvent evtBase
			= caerPoint3DEventPacketGetEvent(reinterpret_cast<caerPoint3DEventPacket>(header), index);
		Point3DEvent *evt = static_cast<Point3DEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerPoint3DEventConst evtBase
			= caerPoint3DEventPacketGetEventConst(reinterpret_cast<caerPoint3DEventPacketConst>(header), index);
		const Point3DEvent *evt = static_cast<const Point3DEvent *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_POINT3D_HPP_ */
