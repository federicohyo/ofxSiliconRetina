#ifndef LIBCAER_DEVICES_DVS132S_HPP_
#define LIBCAER_DEVICES_DVS132S_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"
#include <libcaer/devices/dvs132s.h>
#include "usb.hpp"

namespace libcaer {
namespace devices {

class dvs132s : public usb {
public:
	dvs132s(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_DVS132S) {
	}

	dvs132s(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict)
		: usb(deviceID, CAER_DEVICE_DVS132S, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dvs132s_info infoGet() const noexcept {
		return (caerDVS132SInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
}
}

#endif /* LIBCAER_DEVICES_DVS132S_HPP_ */
