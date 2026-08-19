#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
  I2C_BUS_OK = 0,
  I2C_BUS_ERROR,
  I2C_BUS_TIMEOUT
} I2cBus_Status;

typedef struct I2cBus I2cBus;

struct I2cBus
{
  void *context;
  I2cBus_Status (*read_registers)(void *context, uint8_t address,
                                  uint8_t reg, uint8_t *data, size_t length);
  I2cBus_Status (*write_registers)(void *context, uint8_t address,
                                   uint8_t reg, const uint8_t *data,
                                   size_t length);
  void (*delay_ms)(uint32_t delay_ms);
  /** Reset the controller and release a bus left busy by an interrupted transfer. */
  bool (*recover)(void *context);
};

static inline bool I2cBus_Recover(const I2cBus *bus)
{
  return (bus != NULL) && (bus->recover != NULL) && bus->recover(bus->context);
}

#endif /* I2C_BUS_H */
