#include "byte_ring_buffer.h"

#include <stddef.h>

bool ByteRingBuffer_Init(ByteRingBuffer *buffer, uint8_t *storage,
                         uint16_t capacity)
{
  if ((buffer == NULL) || (storage == NULL) || (capacity < 2U))
  {
    return false;
  }
  buffer->storage = storage;
  buffer->capacity = capacity;
  buffer->write_index = 0U;
  buffer->read_index = 0U;
  buffer->overflow_count = 0U;
  return true;
}

void ByteRingBuffer_Clear(ByteRingBuffer *buffer)
{
  if (buffer != NULL)
  {
    buffer->read_index = 0U;
    buffer->write_index = 0U;
  }
}

bool ByteRingBuffer_Push(ByteRingBuffer *buffer, uint8_t byte)
{
  if ((buffer == NULL) || (buffer->storage == NULL))
  {
    return false;
  }
  const uint16_t next =
      (uint16_t)((buffer->write_index + 1U) % buffer->capacity);
  if (next == buffer->read_index)
  {
    ++buffer->overflow_count;
    return false;
  }
  buffer->storage[buffer->write_index] = byte;
  buffer->write_index = next;
  return true;
}

bool ByteRingBuffer_Pop(ByteRingBuffer *buffer, uint8_t *byte)
{
  if ((buffer == NULL) || (byte == NULL) ||
      (buffer->read_index == buffer->write_index))
  {
    return false;
  }
  *byte = buffer->storage[buffer->read_index];
  buffer->read_index =
      (uint16_t)((buffer->read_index + 1U) % buffer->capacity);
  return true;
}
