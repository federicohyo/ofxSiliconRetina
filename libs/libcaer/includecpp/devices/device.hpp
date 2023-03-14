#ifndef LIBCAER_DEVICES_DEVICE_HPP_
#define LIBCAER_DEVICES_DEVICE_HPP_

#include "../libcaer.hpp"
#include "../events/packetContainer.hpp"
#include "../events/utils.hpp"
#include <libcaer/devices/device.h>
#include <memory>
#include <string>

namespace libcaer {
namespace devices {

class device {
protected:
	std::shared_ptr<struct caer_device_handle> handle;

	device() = default;

public:
	virtual ~device() = default;

	virtual std::string toString() const noexcept = 0;

	void sendDefaultConfig() const {
		bool success = caerDeviceSendDefaultConfig(handle.get());
		if (!success) {
			std::string exc = toString() + ": failed to send default configuration.";
			throw std::runtime_error(exc);
		}
	}

	void configSet(int8_t modAddr, uint8_t paramAddr, uint32_t param) const {
		bool success = caerDeviceConfigSet(handle.get(), modAddr, paramAddr, param);
		if (!success) {
			std::string exc = toString() + ": failed to set configuration parameter, modAddr=" + std::to_string(modAddr)
							  + ", paramAddr=" + std::to_string(paramAddr) + ", param=" + std::to_string(param) + ".";
			throw std::runtime_error(exc);
		}
	}

	void configGet(int8_t modAddr, uint8_t paramAddr, uint32_t *param) const {
		bool success = caerDeviceConfigGet(handle.get(), modAddr, paramAddr, param);
		if (!success) {
			std::string exc = toString() + ": failed to get configuration parameter, modAddr=" + std::to_string(modAddr)
							  + ", paramAddr=" + std::to_string(paramAddr) + ".";
			throw std::runtime_error(exc);
		}
	}

	uint32_t configGet(int8_t modAddr, uint8_t paramAddr) const {
		uint32_t param = 0;
		configGet(modAddr, paramAddr, &param);
		return (param);
	}

	void dataStart(void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
		void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) const {
		bool success = caerDeviceDataStart(handle.get(), dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr,
			dataShutdownNotify, dataShutdownUserPtr);
		if (!success) {
			std::string exc = toString() + ": failed to start getting data.";
			throw std::runtime_error(exc);
		}
	}

	void dataStop() const {
		bool success = caerDeviceDataStop(handle.get());
		if (!success) {
			std::string exc = toString() + ": failed to stop getting data.";
			throw std::runtime_error(exc);
		}
	}

	std::unique_ptr<libcaer::events::EventPacketContainer> dataGet() const {
		caerEventPacketContainer cContainer = caerDeviceDataGet(handle.get());
		if (cContainer == nullptr) {
			// NULL return means no data, forward that.
			return (nullptr);
		}

		std::unique_ptr<libcaer::events::EventPacketContainer> cppContainer
			= std::unique_ptr<libcaer::events::EventPacketContainer>(
				new libcaer::events::EventPacketContainer(cContainer));

		// Free original C container. The event packet memory is now managed by
		// the EventPacket classes inside the new C++ EventPacketContainer.
		free(cContainer);

		return (cppContainer);
	}
};
}
}

#endif /* LIBCAER_DEVICES_DEVICE_HPP_ */
