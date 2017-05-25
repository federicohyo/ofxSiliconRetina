#ifndef LIBCAER_EVENTS_CONFIG_HPP_
#define LIBCAER_EVENTS_CONFIG_HPP_

#include <libcaer/events/config.h>
#include "common.hpp"

namespace libcaer {
namespace events {

struct ConfigurationEvent: public caer_configuration_event {
	int32_t getTimestamp() const noexcept {
		return (caerConfigurationEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerConfigurationEventGetTimestamp64(this,
			reinterpret_cast<caerConfigurationEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTimestamp(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerConfigurationEventSetTimestamp(this, ts);
	}

	bool isValid() const noexcept {
		return (caerConfigurationEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerConfigurationEventValidate(this, reinterpret_cast<caerConfigurationEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerConfigurationEventInvalidate(this,
			reinterpret_cast<caerConfigurationEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getModuleAddress() const noexcept {
		return (caerConfigurationEventGetModuleAddress(this));
	}

	void setModuleAddress(uint8_t modAddr) noexcept {
		caerConfigurationEventSetModuleAddress(this, modAddr);
	}

	uint8_t getParameterAddress() const noexcept {
		return (caerConfigurationEventGetParameterAddress(this));
	}

	void setParameterAddress(uint8_t paramAddr) noexcept {
		caerConfigurationEventSetParameterAddress(this, paramAddr);
	}

	uint32_t getParameter() const noexcept {
		return (caerConfigurationEventGetParameter(this));
	}

	void setParameter(uint32_t param) noexcept {
		caerConfigurationEventSetParameter(this, param);
	}
};

static_assert(std::is_pod<ConfigurationEvent>::value, "ConfigurationEvent is not POD.");

class ConfigurationEventPacket: public EventPacketCommon<ConfigurationEventPacket, ConfigurationEvent> {
public:
	// Constructors.
	ConfigurationEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		caerConfigurationEventPacket packet = caerConfigurationEventPacketAllocate(eventCapacity, eventSource,
			tsOverflow);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	ConfigurationEventPacket(caerConfigurationEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, CONFIG_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	ConfigurationEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, CONFIG_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerConfigurationEvent evtBase = caerConfigurationEventPacketGetEvent(
			reinterpret_cast<caerConfigurationEventPacket>(header), index);
		ConfigurationEvent *evt = static_cast<ConfigurationEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerConfigurationEventConst evtBase = caerConfigurationEventPacketGetEventConst(
			reinterpret_cast<caerConfigurationEventPacketConst>(header), index);
		const ConfigurationEvent *evt = static_cast<const ConfigurationEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_CONFIG_HPP_ */
