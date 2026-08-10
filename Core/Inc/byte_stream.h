#ifndef BYTE_STREAM_H
#define BYTE_STREAM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  void *context;
  bool (*read_byte)(void *context, uint8_t *byte);
} ByteStream;

#endif /* BYTE_STREAM_H */
