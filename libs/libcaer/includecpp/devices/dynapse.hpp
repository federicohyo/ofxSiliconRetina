#ifndef LIBCAER_DEVICES_DYNAPSE_HPP_
#define LIBCAER_DEVICES_DYNAPSE_HPP_

#include "../events/special.hpp"
#include "../events/spike.hpp"
#include <libcaer/devices/dynapse.h>
#include "usb.hpp"

namespace libcaer {
namespace devices {

class dynapse : public usb {
public:
	dynapse(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_DYNAPSE) {
	}

	dynapse(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict)
		: usb(deviceID, CAER_DEVICE_DYNAPSE, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dynapse_info infoGet() const noexcept {
		return (caerDynapseInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}

	void sendDataToUSB(const uint32_t *data, size_t numConfig) const {
		bool success = caerDynapseSendDataToUSB(handle.get(), data, numConfig);
		if (!success) {
			std::string exc = toString() + ": failed to send USB config data to device, numConfig="
							  + std::to_string(numConfig) + ".";
			throw std::runtime_error(exc);
		}
	}

	void writeSramWords(const uint16_t *data, uint32_t baseAddr, size_t numWords) const {
		bool success = caerDynapseWriteSramWords(handle.get(), data, baseAddr, numWords);
		if (!success) {
			std::string exc = toString() + ": failed to write SRAM words to FPGA SRAM, baseAddr="
							  + std::to_string(baseAddr) + ", numWords=" + std::to_string(numWords) + ".";
			throw std::runtime_error(exc);
		}
	}

	void writePoissonSpikeRate(uint16_t neuronAddr, float rateHz) const {
		bool success = caerDynapseWritePoissonSpikeRate(handle.get(), neuronAddr, rateHz);
		if (!success) {
			std::string exc
				= toString() + ": failed to write Poisson Spike Rate, rateHz=" + std::to_string(rateHz) + ".";
			throw std::runtime_error(exc);
		}
	}

	[[deprecated("Replaced by writeSramN(), which has an improved interface.")]] void writeSram(uint8_t coreId,
		uint8_t neuronAddrCore, uint8_t virtualCoreId, bool sx, uint8_t dx, bool sy, uint8_t dy, uint8_t sramId,
		uint8_t destinationCore) const {
		bool success = caerDynapseWriteSram(
			handle.get(), coreId, neuronAddrCore, virtualCoreId, sx, dx, sy, dy, sramId, destinationCore);
		if (!success) {
			std::string exc = toString() + ": failed to write on-chip SRAM, coreId=" + std::to_string(coreId)
							  + ", neuronAddrCore=" + std::to_string(neuronAddrCore) + +", sramId="
							  + std::to_string(sramId) + ", virtualCoreId=" + std::to_string(virtualCoreId)
							  + ", destinationCore=" + std::to_string(destinationCore) + ".";
			throw std::runtime_error(exc);
		}
	}

	void writeSramN(uint16_t neuronAddr, uint8_t sramId, uint8_t virtualCoreId, bool sx, uint8_t dx, bool sy,
		uint8_t dy, uint8_t destinationCore) const {
		bool success
			= caerDynapseWriteSramN(handle.get(), neuronAddr, sramId, virtualCoreId, sx, dx, sy, dy, destinationCore);
		if (!success) {
			std::string exc = toString() + ": failed to write on-chip SRAM, neuronAddr=" + std::to_string(neuronAddr)
							  + ", sramId=" + std::to_string(sramId)
							  + ", virtualCoreId=" + std::to_string(virtualCoreId)
							  + ", destinationCore=" + std::to_string(destinationCore) + ".";
			throw std::runtime_error(exc);
		}
	}

	void writeCam(uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId, uint8_t synapseType) const {
		bool success = caerDynapseWriteCam(handle.get(), inputNeuronAddr, neuronAddr, camId, synapseType);
		if (!success) {
			std::string exc = toString()
							  + ": failed to write on-chip CAM, inputNeuronAddr=" + std::to_string(inputNeuronAddr)
							  + ", neuronAddr=" + std::to_string(neuronAddr) + ", camId=" + std::to_string(camId)
							  + ", synapseType=" + std::to_string(synapseType) + ".";
			throw std::runtime_error(exc);
		}
	}

	// STATIC.
	static uint32_t biasDynapseGenerate(const struct caer_bias_dynapse dynapseBias) noexcept {
		return (caerBiasDynapseGenerate(dynapseBias));
	}

	static struct caer_bias_dynapse biasDynapseParse(const uint32_t dynapseBias) noexcept {
		return (caerBiasDynapseParse(dynapseBias));
	}

	static uint32_t generateCamBits(
		uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId, uint8_t synapseType) noexcept {
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
		struct caer_spike_event sp         = caerDynapseSpikeEventFromXY(x, y);
		libcaer::events::SpikeEvent *spCpp = reinterpret_cast<libcaer::events::SpikeEvent *>(&sp);
		return (*spCpp);
	}
};
}
}

#endif /* LIBCAER_DEVICES_DYNAPSE_HPP_ */
