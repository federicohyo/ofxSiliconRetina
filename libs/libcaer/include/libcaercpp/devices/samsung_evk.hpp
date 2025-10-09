#ifndef LIBCAER_DEVICES_SAMSUNG_EVK_HPP_
#define LIBCAER_DEVICES_SAMSUNG_EVK_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"

#include "../../libcaer/devices/samsung_evk.h"

#include "usb.hpp"

namespace libcaer {
namespace devices {

class samsungEVK : public usb {
public:
	samsungEVK(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_SAMSUNG_EVK) {
	}

	samsungEVK(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, CAER_DEVICE_SAMSUNG_EVK, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_samsung_evk_info infoGet() const noexcept {
		return (caerSamsungEVKInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_SAMSUNG_EVK_HPP_ */
