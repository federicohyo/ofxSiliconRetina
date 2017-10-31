#ifndef LIBCAER_DEVICES_EDVS_HPP_
#define LIBCAER_DEVICES_EDVS_HPP_

#include <libcaer/devices/edvs.h>
#include "serial.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"

namespace libcaer {
namespace devices {

class edvs final: public serial {
public:
	edvs(uint16_t deviceID, const std::string &serialPortName, uint32_t serialBaudRate) :
			serial(deviceID, CAER_DEVICE_EDVS, serialPortName, serialBaudRate) {
	}

	struct caer_edvs_info infoGet() const noexcept {
		return (caerEDVSInfoGet(handle.get()));
	}
};

}
}

#endif /* LIBCAER_DEVICES_EDVS_HPP_ */
