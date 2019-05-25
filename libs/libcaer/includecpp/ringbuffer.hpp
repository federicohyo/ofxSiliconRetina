#ifndef LIBCAER_RINGBUFFER_HPP_
#define LIBCAER_RINGBUFFER_HPP_

#include <libcaer/ringbuffer.h>
#include <memory>
#include <stdexcept>

namespace libcaer {
namespace ringbuffer {

class RingBuffer {
private:
	std::shared_ptr<struct caer_ring_buffer> ringBuffer;

public:
	RingBuffer(size_t size) {
		caerRingBuffer rBuf = caerRingBufferInit(size);

		// Handle constructor failure.
		if (rBuf == nullptr) {
			throw std::runtime_error("Failed to initialize ring-buffer.");
		}

		// Use stateless lambda for shared_ptr custom deleter.
		auto deleteRingBuffer = [](caerRingBuffer rbf) {
			// Run destructor, free all memory.
			// Never fails in current implementation.
			caerRingBufferFree(rbf);
		};

		ringBuffer = std::shared_ptr<struct caer_ring_buffer>(rBuf, deleteRingBuffer);
	}

	void put(void *elem) const {
		bool success = caerRingBufferPut(ringBuffer.get(), elem);
		if (!success) {
			throw std::runtime_error("Failed to put element on ring-buffer.");
		}
	}

	bool full() const noexcept {
		return (caerRingBufferFull(ringBuffer.get()));
	}

	void *get() const noexcept {
		return (caerRingBufferGet(ringBuffer.get()));
	}

	void *look() const noexcept {
		return (caerRingBufferLook(ringBuffer.get()));
	}
};
}
}

#endif /* LIBCAER_RINGBUFFER_HPP_ */
