#ifndef LIBCAER_RINGBUFFER_H_
#define LIBCAER_RINGBUFFER_H_

#ifdef __cplusplus

#include <cstdint>
#include <cstdlib>

#else

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct caer_ring_buffer *caerRingBuffer;

caerRingBuffer caerRingBufferInit(size_t size);
void caerRingBufferFree(caerRingBuffer rBuf);
bool caerRingBufferPut(caerRingBuffer rBuf, void *elem);
bool caerRingBufferFull(caerRingBuffer rBuf);
void *caerRingBufferGet(caerRingBuffer rBuf);
void *caerRingBufferLook(caerRingBuffer rBuf);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_RINGBUFFER_H_ */
