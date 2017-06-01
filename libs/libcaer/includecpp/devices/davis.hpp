#ifndef LIBCAER_DEVICES_DAVIS_HPP_
#define LIBCAER_DEVICES_DAVIS_HPP_

#include <libcaer/devices/davis.h>
#include "usb.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"
#include "../events/frame.hpp"
#include "../events/imu6.hpp"
#include "../events/sample.hpp"

namespace libcaer {
namespace devices {

class davis: public usb {
protected:
	// Forward construction to base class.
	davis(uint16_t deviceID, uint16_t deviceType) :
			usb(deviceID, deviceType) {
	}

	davis(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			usb(deviceID, deviceType, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

public:
	davis(uint16_t deviceID) :
			usb(deviceID, CAER_DEVICE_DAVIS) {
	}

	davis(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			usb(deviceID, CAER_DEVICE_DAVIS, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_davis_info infoGet() const noexcept {
		return (caerDavisInfoGet(handle.get()));
	}

	// STATIC.
	static uint16_t biasVDACGenerate(const struct caer_bias_vdac vdacBias) noexcept {
		return (caerBiasVDACGenerate(vdacBias));
	}

	static struct caer_bias_vdac biasVDACParse(const uint16_t vdacBias) noexcept {
		return (caerBiasVDACParse(vdacBias));
	}

	static uint16_t biasCoarseFineGenerate(const struct caer_bias_coarsefine coarseFineBias) noexcept {
		return (caerBiasCoarseFineGenerate(coarseFineBias));
	}

	static struct caer_bias_coarsefine biasCoarseFineParse(const uint16_t coarseFineBias) noexcept {
		return (caerBiasCoarseFineParse(coarseFineBias));
	}

	static uint16_t biasShiftedSourceGenerate(const struct caer_bias_shiftedsource shiftedSourceBias) noexcept {
		return (caerBiasShiftedSourceGenerate(shiftedSourceBias));
	}

	static struct caer_bias_shiftedsource biasShiftedSourceParse(const uint16_t shiftedSourceBias) noexcept {
		return (caerBiasShiftedSourceParse(shiftedSourceBias));
	}
};

class davisfx2 final: public davis {
public:
	davisfx2(uint16_t deviceID) :
			davis(deviceID, CAER_DEVICE_DAVIS_FX2) {
	}

	davisfx2(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			davis(deviceID, CAER_DEVICE_DAVIS_FX2, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}
};

class davisfx3 final: public davis {
public:
	davisfx3(uint16_t deviceID) :
			davis(deviceID, CAER_DEVICE_DAVIS_FX3) {
	}

	davisfx3(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			davis(deviceID, CAER_DEVICE_DAVIS_FX3, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}
};

}
}

#endif /* LIBCAER_DEVICES_DAVIS_HPP_ */
