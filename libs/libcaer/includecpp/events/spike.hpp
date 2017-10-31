#ifndef LIBCAER_EVENTS_SPIKE_HPP_
#define LIBCAER_EVENTS_SPIKE_HPP_

#include <libcaer/events/spike.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct SpikeEvent: public caer_spike_event {
	int32_t getTimestamp() const noexcept {
		return (caerSpikeEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerSpikeEventGetTimestamp64(this,
			reinterpret_cast<caerSpikeEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerSpikeEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerSpikeEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerSpikeEventValidate(this, reinterpret_cast<caerSpikeEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerSpikeEventInvalidate(this, reinterpret_cast<caerSpikeEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getSourceCoreID() const noexcept {
		return (caerSpikeEventGetSourceCoreID(this));
	}

	void setSourceCoreID(uint8_t coreID) noexcept {
		caerSpikeEventSetSourceCoreID(this, coreID);
	}

	uint8_t getChipID() const noexcept {
		return (caerSpikeEventGetChipID(this));
	}

	void setChipID(uint8_t chipID) noexcept {
		caerSpikeEventSetChipID(this, chipID);
	}

	uint32_t getNeuronID() const noexcept {
		return (caerSpikeEventGetNeuronID(this));
	}

	void setNeuronID(uint32_t neuronID) noexcept {
		caerSpikeEventSetNeuronID(this, neuronID);
	}
};

static_assert(std::is_pod<SpikeEvent>::value, "SpikeEvent is not POD.");

class SpikeEventPacket: public EventPacketCommon<SpikeEventPacket, SpikeEvent> {
public:
	// Constructors.
	SpikeEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerSpikeEventPacket packet = caerSpikeEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	SpikeEventPacket(caerSpikeEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, SPIKE_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	SpikeEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, SPIKE_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerSpikeEvent evtBase = caerSpikeEventPacketGetEvent(reinterpret_cast<caerSpikeEventPacket>(header), index);
		SpikeEvent *evt = static_cast<SpikeEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerSpikeEventConst evtBase = caerSpikeEventPacketGetEventConst(
			reinterpret_cast<caerSpikeEventPacketConst>(header), index);
		const SpikeEvent *evt = static_cast<const SpikeEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_SPIKE_HPP_ */
