#ifndef LIBCAER_EVENTS_SPECIAL_HPP_
#define LIBCAER_EVENTS_SPECIAL_HPP_

#include <libcaer/events/special.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct SpecialEvent: public caer_special_event {
	int32_t getTimestamp() const noexcept {
		return (caerSpecialEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerSpecialEventGetTimestamp64(this,
			reinterpret_cast<caerSpecialEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerSpecialEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerSpecialEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerSpecialEventValidate(this, reinterpret_cast<caerSpecialEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerSpecialEventInvalidate(this, reinterpret_cast<caerSpecialEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getType() const noexcept {
		return (caerSpecialEventGetType(this));
	}

	void setType(uint8_t t) noexcept {
		caerSpecialEventSetType(this, t);
	}

	uint32_t getData() const noexcept {
		return (caerSpecialEventGetData(this));
	}

	void setData(uint32_t d) noexcept {
		caerSpecialEventSetData(this, d);
	}
};

static_assert(std::is_pod<SpecialEvent>::value, "SpecialEvent is not POD.");

class SpecialEventPacket: public EventPacketCommon<SpecialEventPacket, SpecialEvent> {
public:
	// Constructors.
	SpecialEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerSpecialEventPacket packet = caerSpecialEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	SpecialEventPacket(caerSpecialEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, SPECIAL_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	SpecialEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, SPECIAL_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerSpecialEvent evtBase = caerSpecialEventPacketGetEvent(reinterpret_cast<caerSpecialEventPacket>(header),
			index);
		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerSpecialEventConst evtBase = caerSpecialEventPacketGetEventConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), index);
		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}

public:
	reference findEventByType(uint8_t type) {
		caerSpecialEvent evtBase = caerSpecialEventPacketFindEventByType(
			reinterpret_cast<caerSpecialEventPacket>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const_reference findEventByType(uint8_t type) const {
		caerSpecialEventConst evtBase = caerSpecialEventPacketFindEventByTypeConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}

	reference findValidEventByType(uint8_t type) {
		caerSpecialEvent evtBase = caerSpecialEventPacketFindValidEventByType(
			reinterpret_cast<caerSpecialEventPacket>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Valid Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const_reference findValidEventByType(uint8_t type) const {
		caerSpecialEventConst evtBase = caerSpecialEventPacketFindValidEventByTypeConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Valid Special Event of particular type not found.");
		}

		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_SPECIAL_HPP_ */
