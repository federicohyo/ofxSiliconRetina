#ifndef LIBCAER_EVENTS_MATRIX4x4_HPP_
#define LIBCAER_EVENTS_MATRIX4x4_HPP_

#include <libcaer/events/matrix4x4.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct Matrix4x4Event: public caer_matrix4x4_event {
	int32_t getTimestamp() const noexcept {
		return (caerMatrix4x4EventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerMatrix4x4EventGetTimestamp64(this,
			reinterpret_cast<caerMatrix4x4EventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerMatrix4x4EventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerMatrix4x4EventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerMatrix4x4EventValidate(this, reinterpret_cast<caerMatrix4x4EventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerMatrix4x4EventInvalidate(this, reinterpret_cast<caerMatrix4x4EventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerMatrix4x4EventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerMatrix4x4EventSetType(this, t));
	}

	int8_t getScale() const noexcept {
		return (caerMatrix4x4EventGetScale(this));
	}

	void setScale(int8_t s) noexcept {
		return (caerMatrix4x4EventSetScale(this, s));
	}

	float getM00() const noexcept {
		return (caerMatrix4x4EventGetM00(this));
	}

	void setM00(float xVal) noexcept {
		return (caerMatrix4x4EventSetM00(this, xVal));
	}

	float getM01() const noexcept {
		return (caerMatrix4x4EventGetM01(this));
	}

	void setM01(float xVal) noexcept {
		return (caerMatrix4x4EventSetM01(this, xVal));
	}

	float getM02() const noexcept {
		return (caerMatrix4x4EventGetM02(this));
	}

	void setM02(float xVal) noexcept {
		return (caerMatrix4x4EventSetM02(this, xVal));
	}

	float getM03() const noexcept {
		return (caerMatrix4x4EventGetM03(this));
	}

	void setM03(float xVal) noexcept {
		return (caerMatrix4x4EventSetM03(this, xVal));
	}

	float getM10() const noexcept {
		return (caerMatrix4x4EventGetM10(this));
	}

	void setM10(float xVal) noexcept {
		return (caerMatrix4x4EventSetM10(this, xVal));
	}

	float getM11() const noexcept {
		return (caerMatrix4x4EventGetM11(this));
	}

	void setM11(float xVal) noexcept {
		return (caerMatrix4x4EventSetM11(this, xVal));
	}

	float getM12() const noexcept {
		return (caerMatrix4x4EventGetM12(this));
	}

	void setM12(float xVal) noexcept {
		return (caerMatrix4x4EventSetM12(this, xVal));
	}

	float getM13() const noexcept {
		return (caerMatrix4x4EventGetM13(this));
	}

	void setM13(float xVal) noexcept {
		return (caerMatrix4x4EventSetM13(this, xVal));
	}

	float getM20() const noexcept {
		return (caerMatrix4x4EventGetM20(this));
	}

	void setM20(float xVal) noexcept {
		return (caerMatrix4x4EventSetM20(this, xVal));
	}

	float getM21() const noexcept {
		return (caerMatrix4x4EventGetM21(this));
	}

	void setM21(float xVal) noexcept {
		return (caerMatrix4x4EventSetM21(this, xVal));
	}

	float getM22() const noexcept {
		return (caerMatrix4x4EventGetM22(this));
	}

	void setM22(float xVal) noexcept {
		return (caerMatrix4x4EventSetM22(this, xVal));
	}

	float getM23() const noexcept {
		return (caerMatrix4x4EventGetM23(this));
	}

	void setM23(float xVal) noexcept {
		return (caerMatrix4x4EventSetM23(this, xVal));
	}

	float getM30() const noexcept {
		return (caerMatrix4x4EventGetM30(this));
	}

	void setM30(float xVal) noexcept {
		return (caerMatrix4x4EventSetM30(this, xVal));
	}

	float getM31() const noexcept {
		return (caerMatrix4x4EventGetM31(this));
	}

	void setM31(float xVal) noexcept {
		return (caerMatrix4x4EventSetM31(this, xVal));
	}

	float getM32() const noexcept {
		return (caerMatrix4x4EventGetM32(this));
	}

	void setM32(float xVal) noexcept {
		return (caerMatrix4x4EventSetM32(this, xVal));
	}

	float getM33() const noexcept {
		return (caerMatrix4x4EventGetM33(this));
	}

	void setM33(float xVal) noexcept {
		return (caerMatrix4x4EventSetM33(this, xVal));
	}
};

static_assert(std::is_pod<Matrix4x4Event>::value, "Matrix4x4Event is not POD.");

class Matrix4x4EventPacket: public EventPacketCommon<Matrix4x4EventPacket, Matrix4x4Event> {
public:
	// Constructors.
	Matrix4x4EventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerMatrix4x4EventPacket packet = caerMatrix4x4EventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	Matrix4x4EventPacket(caerMatrix4x4EventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, MATRIX4x4_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	Matrix4x4EventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, MATRIX4x4_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerMatrix4x4Event evtBase = caerMatrix4x4EventPacketGetEvent(reinterpret_cast<caerMatrix4x4EventPacket>(header),
			index);
		Matrix4x4Event *evt = static_cast<Matrix4x4Event *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerMatrix4x4EventConst evtBase = caerMatrix4x4EventPacketGetEventConst(
			reinterpret_cast<caerMatrix4x4EventPacketConst>(header), index);
		const Matrix4x4Event *evt = static_cast<const Matrix4x4Event *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_MATRIX4x4_HPP_ */
