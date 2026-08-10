#include "sensor_health.h"

#include <stddef.h>

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

void SensorHealth_Init(SensorHealth *health, uint32_t now_ms,
                       uint32_t stale_timeout_ms, uint32_t retry_interval_ms,
                       uint16_t failure_threshold)
{
  if (health == NULL)
  {
    return;
  }
  *health = (SensorHealth){
      .state = SENSOR_HEALTH_STARTING,
      .last_success_ms = now_ms,
      .next_retry_ms = now_ms + stale_timeout_ms,
      .stale_timeout_ms = stale_timeout_ms,
      .retry_interval_ms = retry_interval_ms,
      .failure_threshold = (failure_threshold == 0U) ? 1U : failure_threshold,
  };
}

void SensorHealth_RecordSuccess(SensorHealth *health, uint32_t now_ms)
{
  if (health == NULL)
  {
    return;
  }
  health->state = SENSOR_HEALTH_OK;
  health->last_success_ms = now_ms;
  health->consecutive_failures = 0U;
  health->has_succeeded = true;
}

void SensorHealth_RecordFailure(SensorHealth *health, uint32_t now_ms)
{
  if (health == NULL)
  {
    return;
  }
  ++health->total_failures;
  if (health->consecutive_failures < UINT16_MAX)
  {
    ++health->consecutive_failures;
  }
  if (health->consecutive_failures >= health->failure_threshold)
  {
    SensorHealth_MarkOffline(health, now_ms);
  }
  else
  {
    health->state = SENSOR_HEALTH_DEGRADED;
  }
}

void SensorHealth_MarkOffline(SensorHealth *health, uint32_t now_ms)
{
  if (health == NULL)
  {
    return;
  }
  health->state = SENSOR_HEALTH_OFFLINE;
  health->next_retry_ms = now_ms + health->retry_interval_ms;
}

void SensorHealth_Update(SensorHealth *health, uint32_t now_ms)
{
  if ((health != NULL) && health->has_succeeded &&
      ((health->state == SENSOR_HEALTH_OK) ||
       (health->state == SENSOR_HEALTH_DEGRADED)) &&
      deadline_reached(now_ms, health->last_success_ms + health->stale_timeout_ms))
  {
    health->state = SENSOR_HEALTH_STALE;
    health->next_retry_ms = now_ms;
  }
}

bool SensorHealth_ShouldRetry(const SensorHealth *health, uint32_t now_ms)
{
  return (health != NULL) &&
         ((health->state == SENSOR_HEALTH_STARTING) ||
          (health->state == SENSOR_HEALTH_OFFLINE) ||
          (health->state == SENSOR_HEALTH_STALE)) &&
         deadline_reached(now_ms, health->next_retry_ms);
}

const char *SensorHealth_StateName(SensorHealth_State state)
{
  static const char *const names[] = {
      "STARTING", "OK", "DEGRADED", "STALE", "OFFLINE"};
  return ((uint32_t)state < (sizeof(names) / sizeof(names[0])))
             ? names[state]
             : "UNKNOWN";
}
