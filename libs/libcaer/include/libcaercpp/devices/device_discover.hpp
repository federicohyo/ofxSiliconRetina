#ifndef LIBCAER_DEVICES_DEVICE_DISCOVER_HPP_
#define LIBCAER_DEVICES_DEVICE_DISCOVER_HPP_

#include "../../libcaer/devices/device_discover.h"

#include "davis.hpp"
#include "device.hpp"
#include "dvs128.hpp"
#include "dvs132s.hpp"
#include "dvxplorer.hpp"
#include "dynapse.hpp"
#include "edvs.hpp"
#include "samsung_evk.hpp"

#include <memory>
#include <vector>

namespace libcaer {
namespace devices {

class discover {
public:
	static std::vector<struct caer_device_discovery_result> device(int16_t deviceType) {
		caerDeviceDiscoveryResult discoveredDevices;

		ssize_t result = caerDeviceDiscover(deviceType, &discoveredDevices);

		if (result < 0) {
			throw std::runtime_error("Device Discovery: failed discovery operation.");
		}

		std::vector<struct caer_device_discovery_result> devices;
		devices.reserve(result);

		for (size_t i = 0; i < (size_t) result; i++) {
			devices.push_back(discoveredDevices[i]);
		}

		free(discoveredDevices);

		return (devices);
	}

	static std::vector<struct caer_device_discovery_result> all() {
		return (device(CAER_DEVICE_DISCOVER_ALL));
	}

	static std::unique_ptr<libcaer::devices::device> open(
		uint16_t deviceID, const struct caer_device_discovery_result &discoveredDevice) {
		switch (discoveredDevice.deviceType) {
			case CAER_DEVICE_DVS128: {
				const struct caer_dvs128_info *info = &discoveredDevice.deviceInfo.dvs128Info;
				return (std::unique_ptr<libcaer::devices::dvs128>(new libcaer::devices::dvs128(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_DAVIS_FX2: {
				const struct caer_davis_info *info = &discoveredDevice.deviceInfo.davisInfo;
				return (std::unique_ptr<libcaer::devices::davisfx2>(new libcaer::devices::davisfx2(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_DAVIS_FX3: {
				const struct caer_davis_info *info = &discoveredDevice.deviceInfo.davisInfo;
				return (std::unique_ptr<libcaer::devices::davisfx3>(new libcaer::devices::davisfx3(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_DAVIS: {
				const struct caer_davis_info *info = &discoveredDevice.deviceInfo.davisInfo;
				return (std::unique_ptr<libcaer::devices::davis>(
					new libcaer::devices::davis(deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_DYNAPSE: {
				const struct caer_dynapse_info *info = &discoveredDevice.deviceInfo.dynapseInfo;
				return (std::unique_ptr<libcaer::devices::dynapse>(new libcaer::devices::dynapse(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_EDVS: {
				const struct caer_edvs_info *info = &discoveredDevice.deviceInfo.edvsInfo;
				return (std::unique_ptr<libcaer::devices::edvs>(
					new libcaer::devices::edvs(deviceID, info->serialPortName, info->serialBaudRate)));
				break;
			}

			case CAER_DEVICE_DVS132S: {
				const struct caer_dvs132s_info *info = &discoveredDevice.deviceInfo.dvs132sInfo;
				return (std::unique_ptr<libcaer::devices::dvs132s>(new libcaer::devices::dvs132s(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_DVXPLORER: {
				const struct caer_dvx_info *info = &discoveredDevice.deviceInfo.dvXplorerInfo;
				return (std::unique_ptr<libcaer::devices::dvXplorer>(new libcaer::devices::dvXplorer(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			case CAER_DEVICE_SAMSUNG_EVK: {
				const struct caer_samsung_evk_info *info = &discoveredDevice.deviceInfo.samsungEVKInfo;
				return (std::unique_ptr<libcaer::devices::samsungEVK>(new libcaer::devices::samsungEVK(
					deviceID, info->deviceUSBBusNumber, info->deviceUSBDeviceAddress, "")));
				break;
			}

			default:
				throw std::runtime_error("Device Discovery: cannot open unknown device.");
				break;
		}
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_DEVICE_DISCOVER_HPP_ */
