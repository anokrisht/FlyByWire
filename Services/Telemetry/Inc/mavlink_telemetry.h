#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK_TELEMETRY_H

#include "data_acquisition.h"
#include "mavlink_encoder.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  UART_HandleTypeDef *uart;
  MavlinkEncoder encoder;
  uint32_t last_heartbeat_ms;
  uint32_t last_imu_ms;
  uint32_t last_attitude_ms;
  uint32_t last_pressure_ms;
  uint32_t last_gps_ms;
  uint32_t last_hud_ms;
  uint32_t last_fault_report_ms;
  SensorHealth_State previous_health[3];
  bool health_announced[3];
  bool previous_airspeed_valid;
  bool airspeed_announced;
  bool utc_valid;
  uint64_t utc_base_epoch_ms;
  uint64_t last_gps_epoch_ms;
  uint32_t utc_base_tick_ms;
} MavlinkTelemetry;

void MavlinkTelemetry_Init(MavlinkTelemetry *telemetry,
                           UART_HandleTypeDef *uart, uint32_t now_ms);
void MavlinkTelemetry_Run(MavlinkTelemetry *telemetry,
                          const DataAcquisitionData *data,
                          uint32_t now_ms);

#endif /* MAVLINK_TELEMETRY_H */
