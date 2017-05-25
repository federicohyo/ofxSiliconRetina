#ifndef LIBCAER_DEVICES_DYNAPSE_HPP_
#define LIBCAER_DEVICES_DYNAPSE_HPP_

#include <libcaer/devices/dynapse.h>
#include "usb.hpp"
#include "../events/spike.hpp"
#include "../events/special.hpp"

namespace libcaer {
namespace devices {

class dynapse final: public usb {
public:
	dynapse(uint16_t deviceID) :
			usb(deviceID, CAER_DEVICE_DYNAPSE) {
	}

	dynapse(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			usb(deviceID, CAER_DEVICE_DYNAPSE, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dynapse_info infoGet() const noexcept {
		return (caerDynapseInfoGet(handle.get()));
	}

	void sendDataToUSB(const uint32_t *data, size_t numConfig) const {
		bool success = caerDynapseSendDataToUSB(handle.get(), data, numConfig);
		if (!success) {
			throw std::runtime_error("Failed to send config data to device.");
		}
	}

	void writeSramWords(const uint16_t *data, uint32_t baseAddr, uint32_t numWords) const {
		bool success = caerDynapseWriteSramWords(handle.get(), data, baseAddr, numWords);
		if (!success) {
			throw std::runtime_error("Failed to write SRAM word.");
		}
	}

	void writeSram(uint16_t coreId, uint32_t neuronId, uint16_t virtualCoreId, bool sx, uint8_t dx, bool sy, uint8_t dy,
		uint16_t sramId, uint16_t destinationCore) const {
		bool success = caerDynapseWriteSram(handle.get(), coreId, neuronId, virtualCoreId, sx, dx, sy, dy, sramId,
			destinationCore);
		if (!success) {
			throw std::runtime_error("Failed to write SRAM.");
		}
	}

	void writeCam(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType) const {
		bool success = caerDynapseWriteCam(handle.get(), preNeuronAddr, postNeuronAddr, camId, synapseType);
		if (!success) {
			throw std::runtime_error("Failed to write CAM.");
		}
	}

	uint32_t generateCamBits(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId,
		int16_t synapseType) const {
		return (caerDynapseGenerateCamBits(preNeuronAddr, postNeuronAddr, camId, synapseType));
	}
};

}
}

#endif /* LIBCAER_DEVICES_DYNAPSE_HPP_ */
