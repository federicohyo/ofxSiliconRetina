#ifndef LIBCAER_EVENTS_EAR_HPP_
#define LIBCAER_EVENTS_EAR_HPP_

#include <libcaer/events/ear.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct EarEvent: public caer_ear_event {
	int32_t getTimestamp() const noexcept {
		return (caerEarEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerEarEventGetTimestamp64(this, reinterpret_cast<caerEarEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerEarEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerEarEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerEarEventValidate(this, reinterpret_cast<caerEarEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerEarEventInvalidate(this, reinterpret_cast<caerEarEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getEar() const noexcept {
		return (caerEarEventGetEar(this));
	}

	void setEar(uint8_t e) noexcept {
		caerEarEventSetEar(this, e);
	}

	uint16_t getChannel() const noexcept {
		return (caerEarEventGetChannel(this));
	}

	void setChannel(uint16_t c) noexcept {
		caerEarEventSetChannel(this, c);
	}

	uint8_t getNeuron() const noexcept {
		return (caerEarEventGetNeuron(this));
	}

	void setNeuron(uint8_t n) noexcept {
		caerEarEventSetNeuron(this, n);
	}

	uint8_t getFilter() const noexcept {
		return (caerEarEventGetFilter(this));
	}

	void setFilter(uint8_t f) noexcept {
		caerEarEventSetFilter(this, f);
	}
};

static_assert(std::is_pod<EarEvent>::value, "EarEvent is not POD.");

class EarEventPacket: public EventPacketCommon<EarEventPacket, EarEvent> {
public:
	// Constructors.
	EarEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerEarEventPacket packet = caerEarEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	EarEventPacket(caerEarEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, EAR_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	EarEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, EAR_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerEarEvent evtBase = caerEarEventPacketGetEvent(reinterpret_cast<caerEarEventPacket>(header), index);
		EarEvent *evt = static_cast<EarEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerEarEventConst evtBase = caerEarEventPacketGetEventConst(reinterpret_cast<caerEarEventPacketConst>(header),
			index);
		const EarEvent *evt = static_cast<const EarEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_EAR_HPP_ */
