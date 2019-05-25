#ifndef LIBCAER_EVENTS_POLARITY_HPP_
#define LIBCAER_EVENTS_POLARITY_HPP_

#include <libcaer/events/polarity.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct PolarityEvent : public caer_polarity_event {
	int32_t getTimestamp() const noexcept {
		return (caerPolarityEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerPolarityEventGetTimestamp64(
			this, reinterpret_cast<caerPolarityEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerPolarityEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerPolarityEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerPolarityEventValidate(this, reinterpret_cast<caerPolarityEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerPolarityEventInvalidate(this, reinterpret_cast<caerPolarityEventPacket>(packet.getHeaderPointer()));
	}

	bool getPolarity() const noexcept {
		return (caerPolarityEventGetPolarity(this));
	}

	void setPolarity(bool pol) noexcept {
		caerPolarityEventSetPolarity(this, pol);
	}

	uint16_t getY() const noexcept {
		return (caerPolarityEventGetY(this));
	}

	void setY(uint16_t y) noexcept {
		caerPolarityEventSetY(this, y);
	}

	uint16_t getX() const noexcept {
		return (caerPolarityEventGetX(this));
	}

	void setX(uint16_t x) noexcept {
		caerPolarityEventSetX(this, x);
	}
};

static_assert(std::is_pod<PolarityEvent>::value, "PolarityEvent is not POD.");

class PolarityEventPacket : public EventPacketCommon<PolarityEventPacket, PolarityEvent> {
public:
	// Constructors.
	PolarityEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerPolarityEventPacket packet = caerPolarityEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	PolarityEventPacket(caerPolarityEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, POLARITY_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	PolarityEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, POLARITY_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerPolarityEvent evtBase
			= caerPolarityEventPacketGetEvent(reinterpret_cast<caerPolarityEventPacket>(header), index);
		PolarityEvent *evt = static_cast<PolarityEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerPolarityEventConst evtBase
			= caerPolarityEventPacketGetEventConst(reinterpret_cast<caerPolarityEventPacketConst>(header), index);
		const PolarityEvent *evt = static_cast<const PolarityEvent *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_POLARITY_HPP_ */
