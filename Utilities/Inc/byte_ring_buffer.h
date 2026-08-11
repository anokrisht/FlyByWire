#ifndef BYTE_RING_BUFFER_H
#define BYTE_RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t *storage;
  uint16_t capacity;
  volatile uint16_t write_index;
  volatile uint16_t read_index;
  volatile uint32_t overflow_count;
} ByteRingBuffer;

bool ByteRingBuffer_Init(ByteRingBuffer *buffer, uint8_t *storage,
                         uint16_t capacity);
void ByteRingBuffer_Clear(ByteRingBuffer *buffer);
bool ByteRingBuffer_Push(ByteRingBuffer *buffer, uint8_t byte);
bool ByteRingBuffer_Pop(ByteRingBuffer *buffer, uint8_t *byte);

#endif /* BYTE_RING_BUFFER_H */
