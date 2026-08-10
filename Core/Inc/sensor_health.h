#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SENSOR_HEALTH_STARTING = 0,
  SENSOR_HEALTH_OK,
  SENSOR_HEALTH_DEGRADED,
  SENSOR_HEALTH_STALE,
  SENSOR_HEALTH_OFFLINE
} SensorHealth_State;

typedef struct
{
  SensorHealth_State state;
  uint32_t last_success_ms;
  uint32_t next_retry_ms;
  uint32_t stale_timeout_ms;
  uint32_t retry_interval_ms;
  uint32_t total_failures;
  uint16_t consecutive_failures;
  uint16_t failure_threshold;
  bool has_succeeded;
} SensorHealth;

void SensorHealth_Init(SensorHealth *health, uint32_t now_ms,
                       uint32_t stale_timeout_ms, uint32_t retry_interval_ms,
                       uint16_t failure_threshold);
void SensorHealth_RecordSuccess(SensorHealth *health, uint32_t now_ms);
void SensorHealth_RecordFailure(SensorHealth *health, uint32_t now_ms);
void SensorHealth_MarkOffline(SensorHealth *health, uint32_t now_ms);
void SensorHealth_Update(SensorHealth *health, uint32_t now_ms);
bool SensorHealth_ShouldRetry(const SensorHealth *health, uint32_t now_ms);
const char *SensorHealth_StateName(SensorHealth_State state);

#endif /* SENSOR_HEALTH_H */
