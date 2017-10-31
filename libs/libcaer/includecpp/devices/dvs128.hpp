#ifndef LIBCAER_DEVICES_DVS128_HPP_
#define LIBCAER_DEVICES_DVS128_HPP_

#include <libcaer/devices/dvs128.h>
#include "usb.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"

namespace libcaer {
namespace devices {

class dvs128 final: public usb {
public:
	dvs128(uint16_t deviceID) :
			usb(deviceID, CAER_DEVICE_DVS128) {
	}

	dvs128(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
			usb(deviceID, CAER_DEVICE_DVS128, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dvs128_info infoGet() const noexcept {
		return (caerDVS128InfoGet(handle.get()));
	}
};

}
}

#endif /* LIBCAER_DEVICES_DVS128_HPP_ */
