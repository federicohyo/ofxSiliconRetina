#ifndef LIBCAER_NETWORK_HPP_
#define LIBCAER_NETWORK_HPP_

#include <libcaer/network.h>
#include "libcaer.hpp"

namespace libcaer {
namespace network {

class AEDAT3NetworkHeader: private aedat3_network_header {
public:
	AEDAT3NetworkHeader() {
		magicNumber = AEDAT3_NETWORK_MAGIC_NUMBER;
		sequenceNumber = 0;
		versionNumber = AEDAT3_NETWORK_VERSION;
		formatNumber = 0;
		sourceID = 0;
	}

	AEDAT3NetworkHeader(const uint8_t *h) {
		struct aedat3_network_header header = caerParseNetworkHeader(h);

		magicNumber = header.magicNumber;
		sequenceNumber = header.sequenceNumber;
		versionNumber = header.versionNumber;
		formatNumber = header.formatNumber;
		sourceID = header.sourceID;
	}

	int64_t getMagicNumber() const noexcept {
		return (magicNumber);
	}

	bool checkMagicNumber() const noexcept {
		return (magicNumber == AEDAT3_NETWORK_MAGIC_NUMBER);
	}

	int64_t getSequenceNumber() const noexcept {
		return (sequenceNumber);
	}

	void incrementSequenceNumber() noexcept {
		sequenceNumber++;
	}

	int8_t getVersionNumber() const noexcept {
		return (versionNumber);
	}

	bool checkVersionNumber() const noexcept {
		return (versionNumber == AEDAT3_NETWORK_VERSION);
	}

	int8_t getFormatNumber() const noexcept {
		return (formatNumber);
	}

	void setFormatNumber(int8_t format) noexcept {
		formatNumber = format;
	}

	int16_t getSourceID() const noexcept {
		return (sourceID);
	}

	void setSourceID(int16_t source) noexcept {
		sourceID = source;
	}
};

}
}

#endif /* LIBCAER_NETWORK_HPP_ */
