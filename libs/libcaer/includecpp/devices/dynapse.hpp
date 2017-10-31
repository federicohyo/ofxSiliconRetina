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
			throw std::runtime_error("Failed to send USB config data to device.");
		}
	}

	void writeSramWords(const uint16_t *data, uint32_t baseAddr, size_t numWords) const {
		bool success = caerDynapseWriteSramWords(handle.get(), data, baseAddr, numWords);
		if (!success) {
			throw std::runtime_error("Failed to write SRAM words to FPGA SRAM.");
		}
	}

	void writePoissonSpikeRate(uint16_t neuronAddr, float rateHz) const {
		bool success = caerDynapseWritePoissonSpikeRate(handle.get(), neuronAddr, rateHz);
		if (!success) {
			throw std::runtime_error("Failed to write Poisson Spike Rate.");
		}
	}

	[[deprecated("Replaced by writeSramN(), which has an improved interface.")]]
	void writeSram(uint8_t coreId, uint8_t neuronAddrCore, uint8_t virtualCoreId, bool sx, uint8_t dx, bool sy,
		uint8_t dy, uint8_t sramId, uint8_t destinationCore) const {
		bool success = caerDynapseWriteSram(handle.get(), coreId, neuronAddrCore, virtualCoreId, sx, dx, sy, dy, sramId,
			destinationCore);
		if (!success) {
			throw std::runtime_error("Failed to write on-chip SRAM.");
		}
	}

	void writeSramN(uint16_t neuronAddr, uint8_t sramId, uint8_t virtualCoreId, bool sx, uint8_t dx, bool sy,
		uint8_t dy, uint8_t destinationCore) const {
		bool success = caerDynapseWriteSramN(handle.get(), neuronAddr, sramId, virtualCoreId, sx, dx, sy, dy,
			destinationCore);
		if (!success) {
			throw std::runtime_error("Failed to write on-chip SRAM.");
		}
	}

	void writeCam(uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId, uint8_t synapseType) const {
		bool success = caerDynapseWriteCam(handle.get(), inputNeuronAddr, neuronAddr, camId, synapseType);
		if (!success) {
			throw std::runtime_error("Failed to write on-chip CAM.");
		}
	}

	// STATIC.
	static uint32_t biasDynapseGenerate(const struct caer_bias_dynapse dynapseBias) noexcept {
		return (caerBiasDynapseGenerate(dynapseBias));
	}

	static struct caer_bias_dynapse biasDynapseParse(const uint32_t dynapseBias) noexcept {
		return (caerBiasDynapseParse(dynapseBias));
	}

	static uint32_t generateCamBits(uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId, uint8_t synapseType)
		noexcept {
		return (caerDynapseGenerateCamBits(inputNeuronAddr, neuronAddr, camId, synapseType));
	}

	static uint32_t generateSramBits(uint16_t neuronAddr, uint8_t sramId, uint8_t virtualCoreId, bool sx, uint8_t dx,
		bool sy, uint8_t dy, uint8_t destinationCore) noexcept {
		return (caerDynapseGenerateSramBits(neuronAddr, sramId, virtualCoreId, sx, dx, sy, dy, destinationCore));
	}

	static uint16_t coreXYToNeuronId(uint8_t coreId, uint8_t columnX, uint8_t rowY) noexcept {
		return (caerDynapseCoreXYToNeuronId(coreId, columnX, rowY));
	}

	static uint16_t coreAddrToNeuronId(uint8_t coreId, uint8_t neuronAddrCore) noexcept {
		return (caerDynapseCoreAddrToNeuronId(coreId, neuronAddrCore));
	}

	static uint16_t spikeEventGetX(const libcaer::events::SpikeEvent &event) noexcept {
		return (caerDynapseSpikeEventGetX(static_cast<caerSpikeEventConst>(&event)));
	}

	static uint16_t spikeEventGetY(const libcaer::events::SpikeEvent &event) noexcept {
		return (caerDynapseSpikeEventGetY(static_cast<caerSpikeEventConst>(&event)));
	}

	static uint16_t spikeEventGetX(const libcaer::events::SpikeEvent *event) noexcept {
		return (caerDynapseSpikeEventGetX(static_cast<caerSpikeEventConst>(event)));
	}

	static uint16_t spikeEventGetY(const libcaer::events::SpikeEvent *event) noexcept {
		return (caerDynapseSpikeEventGetY(static_cast<caerSpikeEventConst>(event)));
	}

	static libcaer::events::SpikeEvent spikeEventFromXY(uint16_t x, uint16_t y) noexcept {
		// This is safe because both structs are guaranteed POD.
		struct caer_spike_event sp = caerDynapseSpikeEventFromXY(x, y);
		libcaer::events::SpikeEvent *spCpp = reinterpret_cast<libcaer::events::SpikeEvent *>(&sp);
		return (*spCpp);
	}
};

}
}

#endif /* LIBCAER_DEVICES_DYNAPSE_HPP_ */
