#include "uart_console.h"

#include <limits.h>
#include <string.h>

static UART_HandleTypeDef *console_uart;

void UartConsole_Init(UART_HandleTypeDef *uart)
{
  console_uart = uart;
}

HAL_StatusTypeDef UartConsole_Write(const uint8_t *data, size_t length)
{
  if ((console_uart == NULL) || ((data == NULL) && (length != 0U)))
  {
    return HAL_ERROR;
  }

  while (length != 0U)
  {
    const uint16_t chunk_length =
        (length > UINT16_MAX) ? UINT16_MAX : (uint16_t)length;
    const HAL_StatusTypeDef status = HAL_UART_Transmit(
        console_uart, data, chunk_length, UART_CONSOLE_TIMEOUT_MS);

    if (status != HAL_OK)
    {
      return status;
    }

    data += chunk_length;
    length -= chunk_length;
  }

  return HAL_OK;
}

HAL_StatusTypeDef UartConsole_WriteString(const char *text)
{
  if (text == NULL)
  {
    return HAL_ERROR;
  }

  return UartConsole_Write((const uint8_t *)text, strlen(text));
}

HAL_StatusTypeDef UartConsole_WriteLine(const char *text)
{
  HAL_StatusTypeDef status = UartConsole_WriteString(text);

  if (status == HAL_OK)
  {
    static const uint8_t line_ending[] = {'\r', '\n'};
    status = UartConsole_Write(line_ending, sizeof(line_ending));
  }

  return status;
}

HAL_StatusTypeDef UartConsole_ReadByte(uint8_t *byte, uint32_t timeout_ms)
{
  if ((console_uart == NULL) || (byte == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive(console_uart, byte, 1U, timeout_ms);
}

int __io_putchar(int character)
{
  const uint8_t byte = (uint8_t)character;

  return (UartConsole_Write(&byte, 1U) == HAL_OK) ? character : -1;
}

int __io_getchar(void)
{
  uint8_t byte;

  return (UartConsole_ReadByte(&byte, HAL_MAX_DELAY) == HAL_OK) ? (int)byte : -1;
}
