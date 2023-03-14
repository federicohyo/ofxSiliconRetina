#ifndef LIBCAER_EVENTS_SAMPLE_HPP_
#define LIBCAER_EVENTS_SAMPLE_HPP_

#include <libcaer/events/sample.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct SampleEvent : public caer_sample_event {
	int32_t getTimestamp() const noexcept {
		return (caerSampleEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerSampleEventGetTimestamp64(
			this, reinterpret_cast<caerSampleEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerSampleEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerSampleEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerSampleEventValidate(this, reinterpret_cast<caerSampleEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerSampleEventInvalidate(this, reinterpret_cast<caerSampleEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerSampleEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		return (caerSampleEventSetType(this, t));
	}

	uint32_t getSample() const noexcept {
		return (caerSampleEventGetSample(this));
	}

	void setSample(uint32_t s) noexcept {
		return (caerSampleEventSetSample(this, s));
	}
};

static_assert(std::is_pod<SampleEvent>::value, "SampleEvent is not POD.");

class SampleEventPacket : public EventPacketCommon<SampleEventPacket, SampleEvent> {
public:
	// Constructors.
	SampleEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerSampleEventPacket packet = caerSampleEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header        = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	SampleEventPacket(caerSampleEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, SAMPLE_EVENT);

		header        = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	SampleEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, SAMPLE_EVENT);

		header        = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerSampleEvent evtBase = caerSampleEventPacketGetEvent(reinterpret_cast<caerSampleEventPacket>(header), index);
		SampleEvent *evt        = static_cast<SampleEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerSampleEventConst evtBase
			= caerSampleEventPacketGetEventConst(reinterpret_cast<caerSampleEventPacketConst>(header), index);
		const SampleEvent *evt = static_cast<const SampleEvent *>(evtBase);

		return (*evt);
	}
};
}
}

#endif /* LIBCAER_EVENTS_SAMPLE_HPP_ */
