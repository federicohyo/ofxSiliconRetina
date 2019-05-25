#ifndef LIBCAER_DEVICES_EDVS_HPP_
#define LIBCAER_DEVICES_EDVS_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"
#include <libcaer/devices/edvs.h>
#include "serial.hpp"

namespace libcaer {
namespace devices {

class edvs : public serial {
public:
	edvs(uint16_t deviceID, const std::string &serialPortName, uint32_t serialBaudRate)
		: serial(deviceID, CAER_DEVICE_EDVS, serialPortName, serialBaudRate) {
	}

	struct caer_edvs_info infoGet() const noexcept {
		return (caerEDVSInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
}
}

#endif /* LIBCAER_DEVICES_EDVS_HPP_ */
