#ifndef LIBCAER_FILTERS_DVS_NOISE_HPP_
#define LIBCAER_FILTERS_DVS_NOISE_HPP_

#include "../events/polarity.hpp"
#include <libcaer/filters/dvs_noise.h>
#include <memory>
#include <string>
#include <vector>

namespace libcaer {
namespace filters {

class DVSNoise {
private:
	std::shared_ptr<struct caer_filter_dvs_noise> handle;

public:
	DVSNoise(uint16_t sizeX, uint16_t sizeY) {
		caerFilterDVSNoise h = caerFilterDVSNoiseInitialize(sizeX, sizeY);

		// Handle constructor failure.
		if (h == nullptr) {
			std::string exc = "Failed to initialize DVS Noise filter, sizeX=" + std::to_string(sizeX)
							  + ", sizeY=" + std::to_string(sizeY) + ".";
			throw std::runtime_error(exc);
		}

		// Use stateless lambda for shared_ptr custom deleter.
		auto deleteDeviceHandle = [](caerFilterDVSNoise fh) {
			// Run destructor, free all memory.
			// Never fails in current implementation.
			caerFilterDVSNoiseDestroy(fh);
		};

		handle = std::shared_ptr<struct caer_filter_dvs_noise>(h, deleteDeviceHandle);
	}

	~DVSNoise() = default;

	std::string toString() const noexcept {
		return ("DVS Noise filter");
	}

	void configSet(uint8_t paramAddr, uint64_t param) const {
		bool success = caerFilterDVSNoiseConfigSet(handle.get(), paramAddr, param);
		if (!success) {
			std::string exc = toString() + ": failed to set configuration parameter, paramAddr="
							  + std::to_string(paramAddr) + ", param=" + std::to_string(param) + ".";
			throw std::runtime_error(exc);
		}
	}

	void configGet(uint8_t paramAddr, uint64_t *param) const {
		bool success = caerFilterDVSNoiseConfigGet(handle.get(), paramAddr, param);
		if (!success) {
			std::string exc
				= toString() + ": failed to get configuration parameter, paramAddr=" + std::to_string(paramAddr) + ".";
			throw std::runtime_error(exc);
		}
	}

	uint64_t configGet(uint8_t paramAddr) const {
		uint64_t param = 0;
		configGet(paramAddr, &param);
		return (param);
	}

	std::vector<struct caer_filter_dvs_pixel> getHotPixels() const {
		caerFilterDVSPixel hotPixels;
		ssize_t numHotPixels = caerFilterDVSNoiseGetHotPixels(handle.get(), &hotPixels);

		if (numHotPixels < 0) {
			std::string exc = toString() + ": failed to get hot pixels array.";
			throw std::runtime_error(exc);
		}

		std::vector<struct caer_filter_dvs_pixel> pixels;
		pixels.reserve(numHotPixels);

		for (size_t i = 0; i < (size_t) numHotPixels; i++) {
			pixels.push_back(hotPixels[i]);
		}

		free(hotPixels);

		return (pixels);
	}

	void apply(caerPolarityEventPacket polarity) const noexcept {
		caerFilterDVSNoiseApply(handle.get(), polarity);
	}

	void apply(libcaer::events::PolarityEventPacket &polarity) const noexcept {
		caerFilterDVSNoiseApply(handle.get(), (caerPolarityEventPacket) polarity.getHeaderPointer());
	}

	void apply(libcaer::events::PolarityEventPacket *polarity) const noexcept {
		if (polarity != nullptr) {
			caerFilterDVSNoiseApply(handle.get(), (caerPolarityEventPacket) polarity->getHeaderPointer());
		}
	}

	void apply(caerPolarityEventPacketConst polarity) const noexcept {
		caerFilterDVSNoiseStatsApply(handle.get(), polarity);
	}

	void apply(const libcaer::events::PolarityEventPacket &polarity) const noexcept {
		caerFilterDVSNoiseStatsApply(handle.get(), (caerPolarityEventPacketConst) polarity.getHeaderPointer());
	}

	void apply(const libcaer::events::PolarityEventPacket *polarity) const noexcept {
		if (polarity != nullptr) {
			caerFilterDVSNoiseStatsApply(handle.get(), (caerPolarityEventPacketConst) polarity->getHeaderPointer());
		}
	}
};
} // namespace filters
} // namespace libcaer

#endif /* LIBCAER_FILTERS_DVS_NOISE_HPP_ */
