#ifndef LIBCAER_DEVICES_USB_HPP_
#define LIBCAER_DEVICES_USB_HPP_

#include <libcaer/devices/usb.h>
#include "device.hpp"

namespace libcaer {
namespace devices {

class usb : public device {
protected:
	usb(uint16_t deviceID, uint16_t deviceType) : usb(deviceID, deviceType, 0, 0, "") {
	}

	usb(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) {
		caerDeviceHandle h = caerDeviceOpen(deviceID, deviceType, busNumberRestrict, devAddressRestrict,
			(serialNumberRestrict.empty()) ? (nullptr) : (serialNumberRestrict.c_str()));

		// Handle constructor failure.
		if (h == nullptr) {
			std::string exc = "Failed to open USB device, id=" + std::to_string(deviceID) + ", type="
							  + std::to_string(deviceType) + ", busNumber=" + std::to_string(busNumberRestrict)
							  + ", devAddress=" + std::to_string(devAddressRestrict)
							  + ", serialNumber=" + serialNumberRestrict + ".";
			throw std::runtime_error(exc);
		}

		// Use stateless lambda for shared_ptr custom deleter.
		auto deleteDeviceHandle = [](caerDeviceHandle cdh) {
			// Run destructor, free all memory.
			// Never fails in current implementation.
			caerDeviceClose(&cdh);
		};

		handle = std::shared_ptr<struct caer_device_handle>(h, deleteDeviceHandle);
	}
};
}
}

#endif /* LIBCAER_DEVICES_USB_HPP_ */
