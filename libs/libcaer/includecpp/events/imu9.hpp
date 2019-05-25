#ifndef LIBCAER_EVENTS_IMU9_HPP_
#define LIBCAER_EVENTS_IMU9_HPP_

#include <libcaer/events/imu9.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct IMU9Event : public caer_imu9_event {
	int32_t getTimestamp() const noexcept {
		return (caerIMU9EventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (
			caerIMU9EventGetTimestamp64(this, reinterpret_cast<caerIMU9EventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerIMU9EventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerIMU9EventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerIMU9EventValidate(this, reinterpret_cast<caerIMU9EventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerIMU9EventInvalidate(this, reinterpret_cast<caerIMU9EventPacket>(packet.getHeaderPointer()));
	}

	float getAccelX() const noexcept {
		return (caerIMU9EventGetAccelX(this));
	}

	void setAccelX(float accelX) noexcept {
		caerIMU9EventSetAccelX(this, accelX);
	}

	float getAccelY() const noexcept {
		return (caerIMU9EventGetAccelY(this));
	}

	void setAccelY(float accelY) noexcept {
		caerIMU9EventSetAccelY(this, accelY);
	}

	float getAccelZ() const noexcept {
		return (caerIMU9EventGetAccelZ(this));
	}

	void setAccelZ(float accelZ) noexcept {
		caerIMU9EventSetAccelZ(this, accelZ);
	}

	float getGyroX() const noexcept {
		return (caerIMU9EventGetGyroX(this));
	}

	void setGyroX(float gyroX) noexcept {
		caerIMU9EventSetGyroX(this, gyroX);
	}

	float getGyroY() const noexcept {
		return (caerIMU9EventGetGyroY(this));
	}

	void setGyroY(float gyroY) noexcept {
		caerIMU9EventSetGyroY(this, gyroY);
	}

	float getGyroZ() const noexcept {
		return (caerIMU9EventGetGyroZ(this));
	}

	void setGyroZ(float gyroZ) noexcept {
		caerIMU9EventSetGyroZ(this, gyroZ);
	}

	float getTemp() const noexcept {
		return (caerIMU9EventGetTemp(this));
	}

	void setTemp(float t) noexcept {
		caerIMU9EventSetTemp(this, t);
	}

	float getCompX() const noexcept {
		return (caerIMU9EventGetCompX(this));
	}

	void setCompX(float compX) noexcept {
		caerIMU9EventSetCompX(this, compX);
	}

	float getCompY() const noexcept {
		return (caerIMU9EventGetCompY(this));
	}

	void setCompY(float compY) noexcept {
		caerIMU9EventSetCompY(this, compY);
	}

	float getCompZ() const noexcept {
		return (caerIMU9EventGetCompZ(this));
	}

	void setCompZ(float compZ) noexcept {
		caerIMU9EventSetCompZ(this, compZ);
	}
};

static_assert(std::is_pod<IMU9Event>::value, "IMU9Event is not POD.");

class IMU9EventPacket : public EventPacketCommon<IMU9EventPacket, IMU9Event> {
public:
	// Constructors.
	IMU9EventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerIMU9EventPacket packet = caerIMU9EventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	IMU9EventPacket(caerIMU9EventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, IMU9_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	IMU9EventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, IMU9_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerIMU9Event evtBase = caerIMU9EventPacketGetEvent(reinterpret_cast<caerIMU9EventPacket>(header), index);
		IMU9Event *evt        = static_cast<IMU9Event *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerIMU9EventConst evtBase
			= caerIMU9EventPacketGetEventConst(reinterpret_cast<caerIMU9EventPacketConst>(header), index);
		const IMU9Event *evt = static_cast<const IMU9Event *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_IMU9_HPP_ */
