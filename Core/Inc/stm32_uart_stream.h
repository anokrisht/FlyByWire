#ifndef STM32_UART_STREAM_H
#define STM32_UART_STREAM_H

#include "byte_stream.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define STM32_UART_STREAM_CAPACITY 256U

typedef struct
{
  UART_HandleTypeDef *handle;
  uint8_t buffer[STM32_UART_STREAM_CAPACITY];
  volatile uint16_t write_index;
  volatile uint16_t read_index;
  volatile uint32_t overflow_count;
  uint8_t interrupt_byte;
  bool started;
} Stm32UartStream;

/** Start continuous interrupt-driven reception and return its generic stream. */
bool Stm32UartStream_Start(Stm32UartStream *stream,
                           UART_HandleTypeDef *handle);
ByteStream Stm32UartStream_AsByteStream(Stm32UartStream *stream);
uint32_t Stm32UartStream_GetOverflowCount(const Stm32UartStream *stream);

#endif /* STM32_UART_STREAM_H */
