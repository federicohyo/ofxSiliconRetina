#ifndef LIBCAER_EVENTS_IMU6_HPP_
#define LIBCAER_EVENTS_IMU6_HPP_

#include <libcaer/events/imu6.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct IMU6Event: public caer_imu6_event {
	int32_t getTimestamp() const noexcept {
		return (caerIMU6EventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerIMU6EventGetTimestamp64(this, reinterpret_cast<caerIMU6EventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerIMU6EventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerIMU6EventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerIMU6EventValidate(this, reinterpret_cast<caerIMU6EventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerIMU6EventInvalidate(this, reinterpret_cast<caerIMU6EventPacket>(packet.getHeaderPointer()));
	}

	float getAccelX() const noexcept {
		return (caerIMU6EventGetAccelX(this));
	}

	void setAccelX(float accelX) noexcept {
		caerIMU6EventSetAccelX(this, accelX);
	}

	float getAccelY() const noexcept {
		return (caerIMU6EventGetAccelY(this));
	}

	void setAccelY(float accelY) noexcept {
		caerIMU6EventSetAccelY(this, accelY);
	}

	float getAccelZ() const noexcept {
		return (caerIMU6EventGetAccelZ(this));
	}

	void setAccelZ(float accelZ) noexcept {
		caerIMU6EventSetAccelZ(this, accelZ);
	}

	float getGyroX() const noexcept {
		return (caerIMU6EventGetGyroX(this));
	}

	void setGyroX(float gyroX) noexcept {
		caerIMU6EventSetGyroX(this, gyroX);
	}

	float getGyroY() const noexcept {
		return (caerIMU6EventGetGyroY(this));
	}

	void setGyroY(float gyroY) noexcept {
		caerIMU6EventSetGyroY(this, gyroY);
	}

	float getGyroZ() const noexcept {
		return (caerIMU6EventGetGyroZ(this));
	}

	void setGyroZ(float gyroZ) noexcept {
		caerIMU6EventSetGyroZ(this, gyroZ);
	}

	float getTemp() const noexcept {
		return (caerIMU6EventGetTemp(this));
	}

	void setTemp(float t) noexcept {
		caerIMU6EventSetTemp(this, t);
	}
};

static_assert(std::is_pod<IMU6Event>::value, "IMU6Event is not POD.");

class IMU6EventPacket: public EventPacketCommon<IMU6EventPacket, IMU6Event> {
public:
	// Constructors.
	IMU6EventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerIMU6EventPacket packet = caerIMU6EventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	IMU6EventPacket(caerIMU6EventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, IMU6_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	IMU6EventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, IMU6_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerIMU6Event evtBase = caerIMU6EventPacketGetEvent(reinterpret_cast<caerIMU6EventPacket>(header), index);
		IMU6Event *evt = static_cast<IMU6Event *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerIMU6EventConst evtBase = caerIMU6EventPacketGetEventConst(
			reinterpret_cast<caerIMU6EventPacketConst>(header), index);
		const IMU6Event *evt = static_cast<const IMU6Event *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_IMU6_HPP_ */
