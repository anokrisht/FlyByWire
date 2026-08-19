#ifndef UART_CONSOLE_H
#define UART_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Default timeout for console transmit and receive operations. */
#define UART_CONSOLE_TIMEOUT_MS 100U

/** Attach the console to an initialized UART peripheral. */
void UartConsole_Init(UART_HandleTypeDef *uart);

/** Enable or silence console output when the UART is used by a binary protocol. */
void UartConsole_SetOutputEnabled(bool enabled);

/** Transmit an arbitrary byte buffer. */
HAL_StatusTypeDef UartConsole_Write(const uint8_t *data, size_t length);

/** Transmit a null-terminated string. */
HAL_StatusTypeDef UartConsole_WriteString(const char *text);

/** Transmit a string followed by CRLF. */
HAL_StatusTypeDef UartConsole_WriteLine(const char *text);

/** Receive one byte, waiting for at most timeout_ms. */
HAL_StatusTypeDef UartConsole_ReadByte(uint8_t *byte, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* UART_CONSOLE_H */
