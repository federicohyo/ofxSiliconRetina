#ifndef LIBCAER_DEVICES_USB_HPP_
#define LIBCAER_DEVICES_USB_HPP_

#include <libcaer/devices/usb.h>
#include <string>
#include "../libcaer.hpp"
#include "../events/packetContainer.hpp"
#include "../events/utils.hpp"

namespace libcaer {
namespace devices {

class usb {
protected:
	std::shared_ptr<struct caer_device_handle> handle;

	usb(uint16_t deviceID, uint16_t deviceType) :
			usb(deviceID, deviceType, 0, 0, "") {
	}

	usb(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) {
		caerDeviceHandle h = caerDeviceOpen(deviceID, deviceType, busNumberRestrict, devAddressRestrict,
			(serialNumberRestrict.empty()) ? (nullptr) : (serialNumberRestrict.c_str()));

		// Handle constructor failure.
		if (h == nullptr) {
			throw std::runtime_error("Failed to open device.");
		}

		// Use stateless lambda for shared_ptr custom deleter.
		auto deleteDeviceHandle = [](caerDeviceHandle h) {
			// Run destructor, free all memory.
			// Never fails in current implementation.
			caerDeviceClose(&h);
		};

		handle = std::shared_ptr<struct caer_device_handle>(h, deleteDeviceHandle);
	}

public:
	void sendDefaultConfig() const {
		bool success = caerDeviceSendDefaultConfig(handle.get());
		if (!success) {
			throw std::runtime_error("Failed to send default configuration.");
		}
	}

	void configSet(int8_t modAddr, uint8_t paramAddr, uint32_t param) const {
		bool success = caerDeviceConfigSet(handle.get(), modAddr, paramAddr, param);
		if (!success) {
			throw std::runtime_error("Failed to set configuration parameter.");
		}
	}

	void configGet(int8_t modAddr, uint8_t paramAddr, uint32_t *param) const {
		bool success = caerDeviceConfigGet(handle.get(), modAddr, paramAddr, param);
		if (!success) {
			throw std::runtime_error("Failed to get configuration parameter.");
		}
	}

	uint32_t configGet(int8_t modAddr, uint8_t paramAddr) const {
		uint32_t param = 0;
		configGet(modAddr, paramAddr, &param);
		return (param);
	}

	void dataStart() const {
		dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);
	}

	void dataStart(void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
		void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) const {
		bool success = caerDeviceDataStart(handle.get(), dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr,
			dataShutdownNotify, dataShutdownUserPtr);
		if (!success) {
			throw std::runtime_error("Failed to start getting data.");
		}
	}

	void dataStop() const {
		bool success = caerDeviceDataStop(handle.get());
		if (!success) {
			throw std::runtime_error("Failed to stop getting data.");
		}
	}

	std::unique_ptr<libcaer::events::EventPacketContainer> dataGet() const {
		caerEventPacketContainer cContainer = caerDeviceDataGet(handle.get());
		if (cContainer == nullptr) {
			// NULL return means no data, forward that.
			return (nullptr);
		}

		std::unique_ptr<libcaer::events::EventPacketContainer> cppContainer = std::unique_ptr<
			libcaer::events::EventPacketContainer>(new libcaer::events::EventPacketContainer());

		for (int32_t i = 0; i < caerEventPacketContainerGetEventPacketsNumber(cContainer); i++) {
			caerEventPacketHeader packet = caerEventPacketContainerGetEventPacket(cContainer, i);

			// NULL packets just get added directly.
			if (packet == nullptr) {
				cppContainer->addEventPacket(nullptr);
			}
			else {
				// Make sure the proper constructors are called when building the shared_ptr.
				cppContainer->addEventPacket(libcaer::events::utils::makeSharedFromCStruct(packet));
			}
		}

		// Free original C container. The event packet memory is now managed by
		// the EventPacket classes inside the new C++ EventPacketContainer.
		free(cContainer);

		return (cppContainer);
	}
};

}
}

#endif /* LIBCAER_DEVICES_USB_HPP_ */
