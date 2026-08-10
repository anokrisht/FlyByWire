#include "stm32_uart_stream.h"

#include <stddef.h>
#include <string.h>

#define REGISTERED_STREAM_COUNT 2U

static Stm32UartStream *registered_streams[REGISTERED_STREAM_COUNT];

static Stm32UartStream *find_stream(UART_HandleTypeDef *handle)
{
  for (uint32_t index = 0U; index < REGISTERED_STREAM_COUNT; ++index)
  {
    if ((registered_streams[index] != NULL) &&
        (registered_streams[index]->handle == handle))
    {
      return registered_streams[index];
    }
  }
  return NULL;
}

static bool register_stream(Stm32UartStream *stream)
{
  for (uint32_t index = 0U; index < REGISTERED_STREAM_COUNT; ++index)
  {
    if ((registered_streams[index] == NULL) ||
        (registered_streams[index] == stream))
    {
      registered_streams[index] = stream;
      return true;
    }
  }
  return false;
}

static bool read_byte(void *context, uint8_t *byte)
{
  Stm32UartStream *stream = context;
  return (stream != NULL) && ByteRingBuffer_Pop(&stream->receive_buffer, byte);
}

bool Stm32UartStream_Start(Stm32UartStream *stream,
                           UART_HandleTypeDef *handle)
{
  if ((stream == NULL) || (handle == NULL))
  {
    return false;
  }

  memset(stream, 0, sizeof(*stream));
  stream->handle = handle;
  if (!ByteRingBuffer_Init(&stream->receive_buffer, stream->buffer,
                           STM32_UART_STREAM_CAPACITY) ||
      !register_stream(stream))
  {
    return false;
  }

  stream->started =
      (HAL_UART_Receive_IT(handle, &stream->interrupt_byte, 1U) == HAL_OK);
  return stream->started;
}

bool Stm32UartStream_Recover(Stm32UartStream *stream)
{
  if ((stream == NULL) || (stream->handle == NULL))
  {
    return false;
  }

  (void)HAL_UART_AbortReceive(stream->handle);
  __HAL_UART_CLEAR_OREFLAG(stream->handle);
  ByteRingBuffer_Clear(&stream->receive_buffer);
  stream->started =
      (HAL_UART_Receive_IT(stream->handle, &stream->interrupt_byte, 1U) == HAL_OK);
  return stream->started;
}

ByteStream Stm32UartStream_AsByteStream(Stm32UartStream *stream)
{
  const ByteStream byte_stream = {
      .context = stream,
      .read_byte = read_byte,
  };
  return byte_stream;
}

uint32_t Stm32UartStream_GetOverflowCount(const Stm32UartStream *stream)
{
  return (stream != NULL) ? stream->receive_buffer.overflow_count : 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
  Stm32UartStream *stream = find_stream(handle);
  if (stream == NULL)
  {
    return;
  }

  (void)ByteRingBuffer_Push(&stream->receive_buffer, stream->interrupt_byte);

  (void)HAL_UART_Receive_IT(handle, &stream->interrupt_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
  Stm32UartStream *stream = find_stream(handle);
  if (stream != NULL)
  {
    (void)HAL_UART_Receive_IT(handle, &stream->interrupt_byte, 1U);
  }
}
