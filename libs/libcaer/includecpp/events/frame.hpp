#ifndef LIBCAER_EVENTS_FRAME_HPP_
#define LIBCAER_EVENTS_FRAME_HPP_

#include <libcaer/events/frame.h>
#include <libcaer/frame_utils.h>
#include "common.hpp"

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

#endif

namespace libcaer {
namespace events {

/**
 * Assignment/Move Constructors/Operators cannot be used with
 * Frame Events due to their particular memory layout that is
 * not entirely known to the compiler (dynamic pixel array).
 * As such those constructors and operators are disabled.
 * Please use caerGenericEventCopy() to copy frame events!
 */
struct FrameEvent: public caer_frame_event {
	FrameEvent() = default;
	FrameEvent(const FrameEvent &rhs) = delete;
	FrameEvent& operator=(const FrameEvent &rhs) = delete;
	FrameEvent(FrameEvent &&rhs) = delete;
	FrameEvent& operator=(FrameEvent &&rhs) = delete;

	enum class colorChannels {
		GRAYSCALE = 1, //!< Grayscale, one channel only.
		RGB = 3,       //!< Red Green Blue, 3 color channels.
		RGBA = 4,      //!< Red Green Blue Alpha, 3 color channels plus transparency.
	};

	enum class colorFilter {
		MONO = 0,    //!< No color filter present, all light passes.
		RGBG = 1,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 1.
		GRGB = 2,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 2.
		GBGR = 3,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 3.
		BGRG = 4,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 4.
		RGBW = 5,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 1.
		GRWB = 6,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 2.
		WBGR = 7,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 3.
		BWRG = 8,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 4.
	};

	int32_t getTSStartOfFrame() const noexcept {
		return (caerFrameEventGetTSStartOfFrame(this));
	}

	int64_t getTSStartOfFrame64(const EventPacket &packet) const noexcept {
		return (caerFrameEventGetTSStartOfFrame64(this,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTSStartOfFrame(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerFrameEventSetTSStartOfFrame(this, ts);
	}

	int32_t getTSEndOfFrame() const noexcept {
		return (caerFrameEventGetTSEndOfFrame(this));
	}

	int64_t getTSEndOfFrame64(const EventPacket &packet) const noexcept {
		return (caerFrameEventGetTSEndOfFrame64(this,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTSEndOfFrame(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerFrameEventSetTSEndOfFrame(this, ts);
	}

	int32_t getTSStartOfExposure() const noexcept {
		return (caerFrameEventGetTSStartOfExposure(this));
	}

	int64_t getTSStartOfExposure64(const EventPacket &packet) const noexcept {
		return (caerFrameEventGetTSStartOfExposure64(this,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTSStartOfExposure(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerFrameEventSetTSStartOfExposure(this, ts);
	}

	int32_t getTSEndOfExposure() const noexcept {
		return (caerFrameEventGetTSEndOfExposure(this));
	}

	int64_t getTSEndOfExposure64(const EventPacket &packet) const noexcept {
		return (caerFrameEventGetTSEndOfExposure64(this,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer())));
	}

	void setTSEndOfExposure(int32_t ts) {
		if (ts < 0) {
			throw std::invalid_argument("Negative timestamp not allowed.");
		}

		caerFrameEventSetTSEndOfExposure(this, ts);
	}

	int32_t getTimestamp() const noexcept {
		return (caerFrameEventGetTimestamp(this));
	}

	int64_t getTimestamp64(const EventPacket &packet) const noexcept {
		return (caerFrameEventGetTimestamp64(this,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer())));
	}

	int32_t getExposureLength() const noexcept {
		return (caerFrameEventGetExposureLength(this));
	}

	bool isValid() const noexcept {
		return (caerFrameEventIsValid(this));
	}

	void validate(EventPacket &packet) noexcept {
		caerFrameEventValidate(this, reinterpret_cast<caerFrameEventPacket>(packet.getHeaderPointer()));
	}

	void invalidate(EventPacket &packet) noexcept {
		caerFrameEventInvalidate(this, reinterpret_cast<caerFrameEventPacket>(packet.getHeaderPointer()));
	}

	uint8_t getROIIdentifier() const noexcept {
		return (caerFrameEventGetROIIdentifier(this));
	}

	void setROIIdentifier(uint8_t roiIdent) noexcept {
		caerFrameEventSetROIIdentifier(this, roiIdent);
	}

	colorFilter getColorFilter() const noexcept {
		return (static_cast<colorFilter>(caerFrameEventGetColorFilter(this)));
	}

	void setColorFilter(colorFilter cFilter) noexcept {
		caerFrameEventSetColorFilter(this,
			static_cast<enum caer_frame_event_color_filter>(static_cast<typename std::underlying_type<colorFilter>::type>(cFilter)));
	}

	int32_t getLengthX() const noexcept {
		return (caerFrameEventGetLengthX(this));
	}

	int32_t getLengthY() const noexcept {
		return (caerFrameEventGetLengthY(this));
	}

	colorChannels getChannelNumber() const noexcept {
		return (static_cast<colorChannels>(caerFrameEventGetChannelNumber(this)));
	}

	void setLengthXLengthYChannelNumber(int32_t lenX, int32_t lenY, colorChannels cNumber, const EventPacket &packet) {
		// Verify lengths and color channels number don't exceed allocated space.
		enum caer_frame_event_color_channels cNumberEnum =
			static_cast<enum caer_frame_event_color_channels>(static_cast<typename std::underlying_type<colorChannels>::type>(cNumber));

		if (lenX <= 0 || lenY <= 0 || cNumberEnum <= 0) {
			throw std::invalid_argument("Negative lengths or channel number not allowed.");
		}

		size_t neededMemory = (sizeof(uint16_t) * static_cast<size_t>(lenX) * static_cast<size_t>(lenY) * cNumberEnum);

		if (neededMemory
			> caerFrameEventPacketGetPixelsSize(
				reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer()))) {
			throw std::invalid_argument("Given values result in memory usage higher than allocated frame event size.");
		}

		caerFrameEventSetLengthXLengthYChannelNumber(this, lenX, lenY, cNumberEnum,
			reinterpret_cast<caerFrameEventPacketConst>(packet.getHeaderPointer()));
	}

	size_t getPixelsMaxIndex() const noexcept {
		return (caerFrameEventGetPixelsMaxIndex(this));
	}

	size_t getPixelsSize() const noexcept {
		return (caerFrameEventGetPixelsSize(this));
	}

	int32_t getPositionX() const noexcept {
		return (caerFrameEventGetPositionX(this));
	}

	void setPositionX(int32_t posX) noexcept {
		caerFrameEventSetPositionX(this, posX);
	}

	int32_t getPositionY() const noexcept {
		return (caerFrameEventGetPositionY(this));
	}

	void setPositionY(int32_t posY) noexcept {
		caerFrameEventSetPositionY(this, posY);
	}

	uint16_t getPixel(int32_t xAddress, int32_t yAddress) const {
		// Check frame bounds first.
		if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
			throw std::invalid_argument("Invalid Y address.");
		}

		int32_t xLength = caerFrameEventGetLengthX(this);

		if (xAddress < 0 || xAddress >= xLength) {
			throw std::invalid_argument("Invalid X address.");
		}

		// Get pixel value at specified position.
		return (le16toh(this->pixels[(yAddress * xLength) + xAddress]));
	}

	void setPixel(int32_t xAddress, int32_t yAddress, uint16_t pixelValue) {
		// Check frame bounds first.
		if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
			throw std::invalid_argument("Invalid Y address.");
		}

		int32_t xLength = caerFrameEventGetLengthX(this);

		if (xAddress < 0 || xAddress >= xLength) {
			throw std::invalid_argument("Invalid X address.");
		}

		// Set pixel value at specified position.
		this->pixels[(yAddress * xLength) + xAddress] = htole16(pixelValue);
	}

	uint16_t getPixel(int32_t xAddress, int32_t yAddress, uint8_t channel) const {
		// Check frame bounds first.
		if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
			throw std::invalid_argument("Invalid Y address.");
		}

		int32_t xLength = caerFrameEventGetLengthX(this);

		if (xAddress < 0 || xAddress >= xLength) {
			throw std::invalid_argument("Invalid X address.");
		}

		uint8_t channelNumber = caerFrameEventGetChannelNumber(this);

		if (channel >= channelNumber) {
			throw std::invalid_argument("Invalid channel number.");
		}

		// Get pixel value at specified position.
		return (le16toh(this->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel]));
	}

	void setPixel(int32_t xAddress, int32_t yAddress, uint8_t channel, uint16_t pixelValue) {
		// Check frame bounds first.
		if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
			throw std::invalid_argument("Invalid Y address.");
		}

		int32_t xLength = caerFrameEventGetLengthX(this);

		if (xAddress < 0 || xAddress >= xLength) {
			throw std::invalid_argument("Invalid X address.");
		}

		uint8_t channelNumber = caerFrameEventGetChannelNumber(this);

		if (channel >= channelNumber) {
			throw std::invalid_argument("Invalid channel number.");
		}

		// Set pixel value at specified position.
		this->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel] = htole16(pixelValue);
	}

	uint16_t getPixelUnsafe(int32_t xAddress, int32_t yAddress) const noexcept {
		// Get pixel value at specified position.
		return (le16toh(this->pixels[(yAddress * caerFrameEventGetLengthX(this)) + xAddress]));
	}

	void setPixelUnsafe(int32_t xAddress, int32_t yAddress, uint16_t pixelValue) noexcept {
		// Set pixel value at specified position.
		this->pixels[(yAddress * caerFrameEventGetLengthX(this)) + xAddress] = htole16(pixelValue);
	}

	uint16_t getPixelUnsafe(int32_t xAddress, int32_t yAddress, uint8_t channel) const noexcept {
		uint8_t channelNumber = caerFrameEventGetChannelNumber(this);
		// Get pixel value at specified position.
		return (le16toh(
			this->pixels[(((yAddress * caerFrameEventGetLengthX(this)) + xAddress) * channelNumber) + channel]));
	}

	void setPixelUnsafe(int32_t xAddress, int32_t yAddress, uint8_t channel, uint16_t pixelValue) noexcept {
		uint8_t channelNumber = caerFrameEventGetChannelNumber(this);
		// Set pixel value at specified position.
		this->pixels[(((yAddress * caerFrameEventGetLengthX(this)) + xAddress) * channelNumber) + channel] = htole16(
			pixelValue);
	}

	uint16_t *getPixelArrayUnsafe() noexcept {
		return (this->pixels);
	}

	const uint16_t *getPixelArrayUnsafe() const noexcept {
		return (this->pixels);
	}

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1

	cv::Mat getOpenCVMat() noexcept {
		const cv::Size frameSize(caerFrameEventGetLengthX(this), caerFrameEventGetLengthY(this));
		cv::Mat frameMat(frameSize, CV_16UC(caerFrameEventGetChannelNumber(this)),
			reinterpret_cast<void *>(this->pixels));
		return (frameMat);
	}

	const cv::Mat getOpenCVMat(bool copyPixels = true) const noexcept {
		const cv::Size frameSize(caerFrameEventGetLengthX(this), caerFrameEventGetLengthY(this));
		const cv::Mat frameMat(frameSize, CV_16UC(caerFrameEventGetChannelNumber(this)),
			reinterpret_cast<void *>(const_cast<uint16_t *>(this->pixels)));

		if (copyPixels) {
			return (frameMat.clone());
		}
		else {
			return (frameMat);
		}
	}

#endif
};

static_assert(std::is_pod<FrameEvent>::value, "FrameEvent is not POD.");

class FrameEventPacket: public EventPacketCommon<FrameEventPacket, FrameEvent> {
public:
	// Constructors.
	FrameEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow, int32_t maxLengthX,
		int32_t maxLengthY, int16_t maxChannelNumber) {
		constructorCheckCapacitySourceTSOverflow(eventCapacity, eventSource, tsOverflow);

		if (maxLengthX <= 0) {
			throw std::invalid_argument("Negative or zero maximum X length not allowed.");
		}
		if (maxLengthY <= 0) {
			throw std::invalid_argument("Negative or zero maximum Y length not allowed.");
		}
		if (maxChannelNumber <= 0) {
			throw std::invalid_argument("Negative or zero maximum number of channels not allowed.");
		}

		caerFrameEventPacket packet = caerFrameEventPacketAllocate(eventCapacity, eventSource, tsOverflow, maxLengthX,
			maxLengthY, maxChannelNumber);
		constructorCheckNullptr(packet);

		header = &packet->packetHeader;
		isMemoryOwner = true; // Always owner on new allocation!
	}

	FrameEventPacket(caerFrameEventPacket packet, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packet);

		constructorCheckEventType(&packet->packetHeader, FRAME_EVENT);

		header = &packet->packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

	FrameEventPacket(caerEventPacketHeader packetHeader, bool takeMemoryOwnership = true) {
		constructorCheckNullptr(packetHeader);

		constructorCheckEventType(packetHeader, FRAME_EVENT);

		header = packetHeader;
		isMemoryOwner = takeMemoryOwnership;
	}

protected:
	// Event access methods.
	reference virtualGetEvent(size_type index) noexcept override {
		caerFrameEvent evtBase = caerFrameEventPacketGetEvent(reinterpret_cast<caerFrameEventPacket>(header), index);
		FrameEvent *evt = static_cast<FrameEvent *>(evtBase);

		return (*evt);
	}

	const_reference virtualGetEvent(size_type index) const noexcept override {
		caerFrameEventConst evtBase = caerFrameEventPacketGetEventConst(
			reinterpret_cast<caerFrameEventPacketConst>(header), index);
		const FrameEvent *evt = static_cast<const FrameEvent *>(evtBase);

		return (*evt);
	}

public:
	size_t getPixelsSize() const noexcept {
		return (caerFrameEventPacketGetPixelsSize(reinterpret_cast<caerFrameEventPacketConst>(header)));
	}

	size_t getPixelsMaxIndex() const noexcept {
		return (caerFrameEventPacketGetPixelsMaxIndex(reinterpret_cast<caerFrameEventPacketConst>(header)));
	}

	enum class demosaicTypes {
		STANDARD = 0,
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		OPENCV_NORMAL = 1,
		OPENCV_EDGE_AWARE = 2,
		// OPENCV_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
#endif
	};

	std::unique_ptr<FrameEventPacket> demosaic(demosaicTypes demosaicType) const {
		caerFrameEventPacket colorPacket =
			caerFrameUtilsDemosaic(reinterpret_cast<caerFrameEventPacketConst>(header),
				static_cast<enum caer_frame_utils_demosaic_types>(static_cast<typename std::underlying_type<
					demosaicTypes>::type>(demosaicType)));
		if (colorPacket == nullptr) {
			throw std::runtime_error("Failed to generate a demosaiced frame event packet.");
		}

		return (std::unique_ptr<FrameEventPacket>(new FrameEventPacket(colorPacket)));
	}

	enum class contrastTypes {
		STANDARD = 0,
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		OPENCV_NORMALIZATION = 1,
		OPENCV_HISTOGRAM_EQUALIZATION = 2,
		OPENCV_CLAHE = 3,
#endif
	};

	void contrast(contrastTypes contrastType) noexcept {
		caerFrameUtilsContrast(reinterpret_cast<caerFrameEventPacket>(header),
			static_cast<enum caer_frame_utils_contrast_types>(static_cast<typename std::underlying_type<contrastTypes>::type>(contrastType)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_FRAME_HPP_ */
