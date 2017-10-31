#include "ringbuffer.h"
#include "portable_aligned_alloc.h"
#include <stdatomic.h>
#include <stdalign.h> // To get alignas() macro.

// Alignment specification support (with defines for cache line alignment).
#if !defined(CACHELINE_SIZE)
#define CACHELINE_SIZE 64 // Default (big enough for most processors), must be power of two!
#endif

struct caer_ring_buffer {
	alignas(CACHELINE_SIZE) size_t putPos;
	uint8_t PAD_putPos[CACHELINE_SIZE - (sizeof(size_t) & (size_t)(CACHELINE_SIZE - 1))];
	alignas(CACHELINE_SIZE) size_t getPos;
	uint8_t PAD_getPos[CACHELINE_SIZE - (sizeof(size_t) & (size_t)(CACHELINE_SIZE - 1))];
	alignas(CACHELINE_SIZE) size_t size;
	uint8_t PAD_size[CACHELINE_SIZE - (sizeof(size_t) & (size_t)(CACHELINE_SIZE - 1))];
	atomic_uintptr_t elements[];
};

caerRingBuffer caerRingBufferInit(size_t size) {
	// Force multiple of two size for performance.
	if ((size == 0) || ((size & (size - 1)) != 0)) {
		return (NULL);
	}

	caerRingBuffer rBuf = portable_aligned_alloc(CACHELINE_SIZE,
		sizeof(struct caer_ring_buffer) + (size * sizeof(atomic_uintptr_t)));
	if (rBuf == NULL) {
		return (NULL);
	}

	// Initialize counter variables.
	rBuf->putPos = 0;
	rBuf->getPos = 0;
	rBuf->size = size;

	// Initialize pointers.
	for (size_t i = 0; i < size; i++) {
		atomic_store_explicit(&rBuf->elements[i], (uintptr_t) NULL, memory_order_relaxed);
	}

	atomic_thread_fence(memory_order_release);

	return (rBuf);
}

void caerRingBufferFree(caerRingBuffer rBuf) {
	portable_aligned_free(rBuf);
}

bool caerRingBufferPut(caerRingBuffer rBuf, void *elem) {
	if (elem == NULL) {
		// NULL elements are disallowed (used as place-holders).
		// Critical error, should never happen -> exit!
		exit(EXIT_FAILURE);
	}

	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->putPos], memory_order_acquire);

	// If the place where we want to put the new element is NULL, it's still
	// free and we can use it.
	if (curr == NULL) {
		atomic_store_explicit(&rBuf->elements[rBuf->putPos], (uintptr_t ) elem, memory_order_release);

		// Increase local put pointer.
		rBuf->putPos = ((rBuf->putPos + 1) & (rBuf->size - 1));

		return (true);
	}

	// Else, buffer is full.
	return (false);
}

bool caerRingBufferFull(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->putPos], memory_order_acquire);

	// If the place where we want to put a new element is not NULL,
	// there is no place to put new data, so the buffer is full.
	if (curr != NULL) {
		return (true);
	}

	// Else, buffer is not full.
	return (false);
}

void *caerRingBufferGet(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->getPos], memory_order_acquire);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, and reset the place to NULL.
	if (curr != NULL) {
		atomic_store_explicit(&rBuf->elements[rBuf->getPos], (uintptr_t) NULL, memory_order_release);

		// Increase local get pointer.
		rBuf->getPos = ((rBuf->getPos + 1) & (rBuf->size - 1));

		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}

void *caerRingBufferLook(caerRingBuffer rBuf) {
	void *curr = (void *) atomic_load_explicit(&rBuf->elements[rBuf->getPos], memory_order_acquire);

	// If the place where we want to get an element from is not NULL, there
	// is valid content there, which we return, without removing it from the
	// ring buffer.
	if (curr != NULL) {
		return (curr);
	}

	// Else, buffer is empty.
	return (NULL);
}
