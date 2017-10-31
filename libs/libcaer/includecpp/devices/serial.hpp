#ifndef LIBCAER_DEVICES_SERIAL_HPP_
#define LIBCAER_DEVICES_SERIAL_HPP_

#include <libcaer/devices/serial.h>
#include "device.hpp"

namespace libcaer {
namespace devices {

class serial: public device {
protected:
	serial(uint16_t deviceID, uint16_t deviceType, const std::string &serialPortName, uint32_t serialBaudRate) {
		caerDeviceHandle h = caerDeviceOpenSerial(deviceID, deviceType, serialPortName.c_str(), serialBaudRate);

		// Handle constructor failure.
		if (h == nullptr) {
			throw std::runtime_error("Failed to open serial port device.");
		}

		// Use stateless lambda for shared_ptr custom deleter.
		auto deleteDeviceHandle = [](caerDeviceHandle h) {
			// Run destructor, free all memory.
			// Never fails in current implementation.
			caerDeviceClose(&h);
		};

		handle = std::shared_ptr<struct caer_device_handle>(h, deleteDeviceHandle);
	}
};

}
}

#endif /* LIBCAER_DEVICES_SERIAL_HPP_ */
