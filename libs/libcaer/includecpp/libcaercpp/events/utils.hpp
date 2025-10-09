#ifndef LIBCAER_EVENTS_UTILS_HPP_
#define LIBCAER_EVENTS_UTILS_HPP_

#include "common.hpp"
#include "frame.hpp"
#include "imu6.hpp"
#include "imu9.hpp"
#include "polarity.hpp"
#include "special.hpp"
#include "spike.hpp"

#include <memory>

namespace libcaer {
namespace events {
namespace utils {

inline std::unique_ptr<EventPacket> makeUniqueFromCStruct(caerEventPacketHeader packet, bool takeMemoryOwnership);
inline std::shared_ptr<EventPacket> makeSharedFromCStruct(caerEventPacketHeader packet, bool takeMemoryOwnership);
inline enum caer_frame_utils_pixel_color getPixelColor(
	libcaer::events::FrameEvent::colorFilter cFilter, int32_t x, int32_t y);
inline enum caer_frame_utils_pixel_color getPixelColor(
	enum caer_frame_event_color_filter cFilter, int32_t x, int32_t y);

inline std::unique_ptr<EventPacket> makeUniqueFromCStruct(
	caerEventPacketHeader packet, bool takeMemoryOwnership = true) {
	switch (caerEventPacketHeaderGetEventType(packet)) {
		case SPECIAL_EVENT:
			return (std::unique_ptr<SpecialEventPacket>(new SpecialEventPacket(packet, takeMemoryOwnership)));
			break;

		case POLARITY_EVENT:
			return (std::unique_ptr<PolarityEventPacket>(new PolarityEventPacket(packet, takeMemoryOwnership)));
			break;

		case FRAME_EVENT:
			return (std::unique_ptr<FrameEventPacket>(new FrameEventPacket(packet, takeMemoryOwnership)));
			break;

		case IMU6_EVENT:
			return (std::unique_ptr<IMU6EventPacket>(new IMU6EventPacket(packet, takeMemoryOwnership)));
			break;

		case IMU9_EVENT:
			return (std::unique_ptr<IMU9EventPacket>(new IMU9EventPacket(packet, takeMemoryOwnership)));
			break;

		case SPIKE_EVENT:
			return (std::unique_ptr<SpikeEventPacket>(new SpikeEventPacket(packet, takeMemoryOwnership)));
			break;

		default:
			return (std::unique_ptr<EventPacket>(new EventPacket(packet, takeMemoryOwnership)));
			break;
	}
}

inline std::shared_ptr<EventPacket> makeSharedFromCStruct(
	caerEventPacketHeader packet, bool takeMemoryOwnership = true) {
	switch (caerEventPacketHeaderGetEventType(packet)) {
		case SPECIAL_EVENT:
			return (std::make_shared<SpecialEventPacket>(packet, takeMemoryOwnership));
			break;

		case POLARITY_EVENT:
			return (std::make_shared<PolarityEventPacket>(packet, takeMemoryOwnership));
			break;

		case FRAME_EVENT:
			return (std::make_shared<FrameEventPacket>(packet, takeMemoryOwnership));
			break;

		case IMU6_EVENT:
			return (std::make_shared<IMU6EventPacket>(packet, takeMemoryOwnership));
			break;

		case IMU9_EVENT:
			return (std::make_shared<IMU9EventPacket>(packet, takeMemoryOwnership));
			break;

		case SPIKE_EVENT:
			return (std::make_shared<SpikeEventPacket>(packet, takeMemoryOwnership));
			break;

		default:
			return (std::make_shared<EventPacket>(packet, takeMemoryOwnership));
			break;
	}
}

inline enum caer_frame_utils_pixel_color getPixelColor(
	libcaer::events::FrameEvent::colorFilter cFilter, int32_t x, int32_t y) {
	return (caerFrameUtilsPixelColor(
		static_cast<enum caer_frame_event_color_filter>(
			static_cast<typename std::underlying_type<libcaer::events::FrameEvent::colorFilter>::type>(cFilter)),
		x, y));
}

inline enum caer_frame_utils_pixel_color getPixelColor(
	enum caer_frame_event_color_filter cFilter, int32_t x, int32_t y) {
	return (caerFrameUtilsPixelColor(cFilter, x, y));
}

} // namespace utils
} // namespace events
} // namespace libcaer

#endif /* LIBCAER_EVENTS_UTILS_HPP_ */
