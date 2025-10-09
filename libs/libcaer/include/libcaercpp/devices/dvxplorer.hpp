#ifndef LIBCAER_DEVICES_DVXPLORER_HPP_
#define LIBCAER_DEVICES_DVXPLORER_HPP_

#include "../events/imu6.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"

#include "../../libcaer/devices/dvxplorer.h"

#include "usb.hpp"

namespace libcaer {
namespace devices {

class dvXplorer : public usb {
public:
	dvXplorer(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_DVXPLORER) {
	}

	dvXplorer(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, CAER_DEVICE_DVXPLORER, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dvx_info infoGet() const noexcept {
		return (caerDVXplorerInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_DVXPLORER_HPP_ */
