#include "mavlink_telemetry.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_SYSTEM_ID             1U
#define MAVLINK_COMPONENT_ID          1U
#define HEARTBEAT_PERIOD_MS         1000U
#define HIGHRES_IMU_PERIOD_MS         20U
#define ATTITUDE_PERIOD_MS            40U
#define PRESSURE_PERIOD_MS            40U
#define GPS_PERIOD_MS                200U
#define HUD_PERIOD_MS                100U
#define FAULT_REPEAT_PERIOD_MS      5000U
#define MAVLINK_UART_TIMEOUT_MS        20U

#define MAVLINK_MSG_ID_HEARTBEAT        0U
#define MAVLINK_MSG_ID_GPS_RAW_INT     24U
#define MAVLINK_MSG_ID_SCALED_PRESSURE 29U
#define MAVLINK_MSG_ID_ATTITUDE        30U
#define MAVLINK_MSG_ID_VFR_HUD         74U
#define MAVLINK_MSG_ID_HIGHRES_IMU    105U
#define MAVLINK_MSG_ID_STATUSTEXT      253U

#define MAVLINK_CRC_HEARTBEAT         50U
#define MAVLINK_CRC_GPS_RAW_INT       24U
#define MAVLINK_CRC_SCALED_PRESSURE  115U
#define MAVLINK_CRC_ATTITUDE          39U
#define MAVLINK_CRC_VFR_HUD           20U
#define MAVLINK_CRC_HIGHRES_IMU       93U
#define MAVLINK_CRC_STATUSTEXT         83U

#define MAV_TYPE_FIXED_WING            1U
#define MAV_AUTOPILOT_GENERIC          0U
#define MAV_STATE_ACTIVE               4U
#define MAVLINK_VERSION                3U
#define MAV_SEVERITY_CRITICAL           2U
#define MAV_SEVERITY_ERROR              3U
#define MAV_SEVERITY_WARNING            4U
#define MAV_SEVERITY_NOTICE             5U
#define MAV_SEVERITY_INFO               6U
#define HIGHRES_IMU_FIELDS          0x11FFU
#define HIGHRES_ABS_PRESSURE_FIELD  0x0200U
#define HIGHRES_DIFF_PRESSURE_FIELD 0x0400U
#define HIGHRES_ALTITUDE_FIELD      0x0800U

static void put_u16(uint8_t *payload, uint8_t offset, uint16_t value)
{
  payload[offset] = (uint8_t)value;
  payload[offset + 1U] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *payload, uint8_t offset, uint32_t value)
{
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    payload[offset + i] = (uint8_t)(value >> (8U * i));
  }
}

static void put_u64(uint8_t *payload, uint8_t offset, uint64_t value)
{
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    payload[offset + i] = (uint8_t)(value >> (8U * i));
  }
}

static void put_float(uint8_t *payload, uint8_t offset, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  put_u32(payload, offset, bits);
}

static void transmit(MavlinkTelemetry *telemetry, uint32_t message_id,
                     uint8_t crc_extra, const uint8_t *payload,
                     uint8_t payload_length)
{
  uint8_t packet[MAVLINK2_MAX_PACKET_LENGTH];
  const size_t length = MavlinkEncoder_Encode(
      &telemetry->encoder, message_id, crc_extra, payload, payload_length,
      packet, sizeof(packet));
  if (length != 0U)
  {
    (void)HAL_UART_Transmit(telemetry->uart, packet, (uint16_t)length,
                            MAVLINK_UART_TIMEOUT_MS);
  }
}

static bool elapsed(uint32_t now, uint32_t *previous, uint32_t period)
{
  if ((uint32_t)(now - *previous) < period)
  {
    return false;
  }
  *previous = now;
  return true;
}

static float value_or_zero(bool valid, float value)
{
  return valid ? value : 0.0F;
}

static uint16_t scaled_u16(float value, float scale, uint16_t invalid)
{
  if (!isfinite(value) || (value < 0.0F))
  {
    return invalid;
  }
  const float scaled = value * scale;
  return (scaled >= (float)UINT16_MAX) ? (UINT16_MAX - 1U) :
                                         (uint16_t)lroundf(scaled);
}

static int32_t scaled_i32(double value, double scale)
{
  const double scaled = value * scale;
  if (scaled >= (double)INT32_MAX) return INT32_MAX;
  if (scaled <= (double)INT32_MIN) return INT32_MIN;
  return (int32_t)llround(scaled);
}

static void send_heartbeat(MavlinkTelemetry *telemetry)
{
  uint8_t payload[9] = {0};
  payload[4] = MAV_TYPE_FIXED_WING;
  payload[5] = MAV_AUTOPILOT_GENERIC;
  payload[6] = 0U;
  payload[7] = MAV_STATE_ACTIVE;
  payload[8] = MAVLINK_VERSION;
  transmit(telemetry, MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_CRC_HEARTBEAT,
           payload, sizeof(payload));
}

static int64_t days_from_civil(int32_t year, uint32_t month, uint32_t day)
{
  year -= (month <= 2U) ? 1 : 0;
  const int32_t era = year / 400;
  const uint32_t year_of_era = (uint32_t)(year - era * 400);
  const uint32_t day_of_year =
      (153U * (month + ((month > 2U) ? (uint32_t)-3 : 9U)) + 2U) / 5U +
      day - 1U;
  const uint32_t day_of_era = year_of_era * 365U + year_of_era / 4U -
                              year_of_era / 100U + day_of_year;
  return (int64_t)era * 146097LL + (int64_t)day_of_era - 719468LL;
}

static void civil_from_days(int64_t days, int32_t *year, uint32_t *month,
                            uint32_t *day)
{
  days += 719468LL;
  const int64_t era = (days >= 0) ? days / 146097LL :
                                    (days - 146096LL) / 146097LL;
  const uint32_t day_of_era = (uint32_t)(days - era * 146097LL);
  const uint32_t year_of_era =
      (day_of_era - day_of_era / 1460U + day_of_era / 36524U -
       day_of_era / 146096U) / 365U;
  int32_t calculated_year = (int32_t)year_of_era + (int32_t)era * 400;
  const uint32_t day_of_year = day_of_era -
      (365U * year_of_era + year_of_era / 4U - year_of_era / 100U);
  const uint32_t month_prime = (5U * day_of_year + 2U) / 153U;
  *day = day_of_year - (153U * month_prime + 2U) / 5U + 1U;
  *month = month_prime + ((month_prime < 10U) ? 3U : (uint32_t)-9);
  calculated_year += (*month <= 2U) ? 1 : 0;
  *year = calculated_year;
}

static bool gps_epoch_ms(const GpsData *gps, uint64_t *epoch_ms)
{
  if ((gps == NULL) || (epoch_ms == NULL) || !gps->utc_date.valid ||
      !gps->utc_time.valid || (gps->utc_date.year < 1970U) ||
      (gps->utc_date.month < 1U) || (gps->utc_date.month > 12U) ||
      (gps->utc_date.day < 1U) || (gps->utc_date.day > 31U) ||
      (gps->utc_time.hours > 23U) || (gps->utc_time.minutes > 59U) ||
      (gps->utc_time.seconds > 60U) ||
      (gps->utc_time.milliseconds > 999U))
  {
    return false;
  }
  const int64_t days = days_from_civil((int32_t)gps->utc_date.year,
                                       gps->utc_date.month, gps->utc_date.day);
  if (days < 0) return false;
  const uint64_t seconds = (uint64_t)days * 86400ULL +
      (uint64_t)gps->utc_time.hours * 3600ULL +
      (uint64_t)gps->utc_time.minutes * 60ULL + gps->utc_time.seconds;
  *epoch_ms = seconds * 1000ULL + gps->utc_time.milliseconds;
  return true;
}

static void update_utc_clock(MavlinkTelemetry *telemetry,
                             const DataAcquisitionData *data,
                             uint32_t now_ms)
{
  uint64_t gps_time_ms;
  if (gps_epoch_ms(&data->gps, &gps_time_ms) &&
      (!telemetry->utc_valid ||
       (gps_time_ms != telemetry->last_gps_epoch_ms)))
  {
    telemetry->utc_valid = true;
    telemetry->utc_base_epoch_ms = gps_time_ms;
    telemetry->last_gps_epoch_ms = gps_time_ms;
    telemetry->utc_base_tick_ms = now_ms;
  }
}

static void format_status_timestamp(const MavlinkTelemetry *telemetry,
                                    uint32_t now_ms, char *output,
                                    size_t output_size)
{
  if (telemetry->utc_valid)
  {
    const uint64_t epoch_ms = telemetry->utc_base_epoch_ms +
        (uint32_t)(now_ms - telemetry->utc_base_tick_ms);
    const uint64_t total_seconds = epoch_ms / 1000ULL;
    int32_t year;
    uint32_t month;
    uint32_t day;
    civil_from_days((int64_t)(total_seconds / 86400ULL), &year, &month, &day);
    const uint32_t seconds_of_day = (uint32_t)(total_seconds % 86400ULL);
    (void)snprintf(output, output_size,
                   "[%04ld-%02lu-%02luT%02lu:%02lu:%02lu.%03luZ]",
                   (long)year, (unsigned long)month, (unsigned long)day,
                   (unsigned long)(seconds_of_day / 3600U),
                   (unsigned long)((seconds_of_day / 60U) % 60U),
                   (unsigned long)(seconds_of_day % 60U),
                   (unsigned long)(epoch_ms % 1000ULL));
    return;
  }

  const uint32_t total_seconds = now_ms / 1000U;
  (void)snprintf(output, output_size, "[T+%02lu:%02lu:%02lu.%03lu]",
                 (unsigned long)(total_seconds / 3600U),
                 (unsigned long)((total_seconds / 60U) % 60U),
                 (unsigned long)(total_seconds % 60U),
                 (unsigned long)(now_ms % 1000U));
}

static void send_status_text(MavlinkTelemetry *telemetry, uint8_t severity,
                             uint32_t timestamp_ms, const char *text)
{
  uint8_t payload[51] = {0};
  payload[0] = severity;
  if (text != NULL)
  {
    char timestamp[32];
    char timestamped[51];
    format_status_timestamp(telemetry, timestamp_ms, timestamp,
                            sizeof(timestamp));
    (void)snprintf(timestamped, sizeof(timestamped), "%s %s", timestamp,
                   text);
    size_t length = strlen(timestamped);
    if (length > 50U) length = 50U;
    memcpy(&payload[1], timestamped, length);
  }
  transmit(telemetry, MAVLINK_MSG_ID_STATUSTEXT, MAVLINK_CRC_STATUSTEXT,
           payload, sizeof(payload));
}

static uint8_t health_severity(SensorHealth_State state)
{
  switch (state)
  {
    case SENSOR_HEALTH_OFFLINE: return MAV_SEVERITY_CRITICAL;
    case SENSOR_HEALTH_STALE: return MAV_SEVERITY_ERROR;
    case SENSOR_HEALTH_DEGRADED: return MAV_SEVERITY_WARNING;
    case SENSOR_HEALTH_STARTING: return MAV_SEVERITY_NOTICE;
    case SENSOR_HEALTH_OK:
    default: return MAV_SEVERITY_INFO;
  }
}

static void report_sensor_health(MavlinkTelemetry *telemetry, uint8_t index,
                                 const char *sensor_name,
                                 const SensorHealth *health,
                                 bool repeat_fault, uint32_t now_ms)
{
  if ((health == NULL) || (index >= 3U)) return;
  const bool unchanged = telemetry->health_announced[index] &&
      (telemetry->previous_health[index] == health->state);
  if (unchanged && (!repeat_fault || (health->state == SENSOR_HEALTH_OK)))
  {
    return;
  }

  char message[51];
  (void)snprintf(message, sizeof(message), "%s: %s", sensor_name,
                 SensorHealth_StateName(health->state));
  send_status_text(telemetry, health_severity(health->state), now_ms, message);
  telemetry->previous_health[index] = health->state;
  telemetry->health_announced[index] = true;
}

static void report_health_changes(MavlinkTelemetry *telemetry,
                                  const DataAcquisitionData *data,
                                  bool repeat_faults, uint32_t now_ms)
{
  report_sensor_health(telemetry, 0U, "IMU", &data->imu_health,
                       repeat_faults, now_ms);
  report_sensor_health(telemetry, 1U, "BAROMETER", &data->barometer_health,
                       repeat_faults, now_ms);
  report_sensor_health(telemetry, 2U, "GPS", &data->gps_health,
                       repeat_faults, now_ms);

  if (!telemetry->airspeed_announced ||
      (telemetry->previous_airspeed_valid != data->airspeed_valid) ||
      (repeat_faults && !data->airspeed_valid))
  {
    send_status_text(telemetry,
                     data->airspeed_valid ? MAV_SEVERITY_INFO :
                                            MAV_SEVERITY_CRITICAL,
                     now_ms,
                     data->airspeed_valid ? "AIRSPEED: OK" :
                                            "AIRSPEED: OFFLINE");
    telemetry->previous_airspeed_valid = data->airspeed_valid;
    telemetry->airspeed_announced = true;
  }
}

static void send_highres_imu(MavlinkTelemetry *telemetry,
                             const DataAcquisitionData *data)
{
  uint8_t payload[62] = {0};
  put_u64(payload, 0U, (uint64_t)data->timestamp_ms * 1000ULL);
  for (uint8_t axis = 0U; axis < 3U; ++axis)
  {
    put_float(payload, (uint8_t)(8U + axis * 4U),
              value_or_zero(data->imu_valid,
                            data->imu.acceleration_mps2[axis]));
    put_float(payload, (uint8_t)(20U + axis * 4U),
              value_or_zero(data->imu_valid,
                            data->imu.angular_rate_rps[axis]));
    put_float(payload, (uint8_t)(32U + axis * 4U),
              value_or_zero(data->imu_valid,
                            data->imu.magnetic_field_ut[axis] * 0.01F));
  }
  put_float(payload, 44U, value_or_zero(data->barometer_valid,
                                        data->barometer.pressure_pa * 0.01F));
  put_float(payload, 48U, value_or_zero(data->airspeed_valid,
                                        data->differential_pressure_pa * 0.01F));
  put_float(payload, 52U, value_or_zero(data->barometer_valid,
                                        data->altitude_m));
  put_float(payload, 56U, value_or_zero(data->imu_valid,
                                        data->imu.temperature_c));
  uint16_t fields_updated = data->imu_valid ? HIGHRES_IMU_FIELDS : 0U;
  if (data->barometer_valid)
  {
    fields_updated |= HIGHRES_ABS_PRESSURE_FIELD | HIGHRES_ALTITUDE_FIELD;
  }
  if (data->airspeed_valid)
  {
    fields_updated |= HIGHRES_DIFF_PRESSURE_FIELD;
  }
  put_u16(payload, 60U, fields_updated);
  transmit(telemetry, MAVLINK_MSG_ID_HIGHRES_IMU, MAVLINK_CRC_HIGHRES_IMU,
           payload, sizeof(payload));
}

static void send_attitude(MavlinkTelemetry *telemetry,
                          const DataAcquisitionData *data)
{
  static const float degrees_to_radians = 0.01745329251994329577F;
  uint8_t payload[28] = {0};
  put_u32(payload, 0U, data->timestamp_ms);
  put_float(payload, 4U, value_or_zero(data->orientation_valid,
                                       data->orientation.roll_deg) *
                          degrees_to_radians);
  put_float(payload, 8U, value_or_zero(data->orientation_valid,
                                       data->orientation.pitch_deg) *
                          degrees_to_radians);
  put_float(payload, 12U, value_or_zero(data->orientation_valid,
                                        data->orientation.yaw_deg) *
                           degrees_to_radians);
  put_float(payload, 16U, value_or_zero(data->imu_valid,
                                        data->imu.angular_rate_rps[0]));
  put_float(payload, 20U, value_or_zero(data->imu_valid,
                                        data->imu.angular_rate_rps[1]));
  put_float(payload, 24U, value_or_zero(data->imu_valid,
                                        data->imu.angular_rate_rps[2]));
  transmit(telemetry, MAVLINK_MSG_ID_ATTITUDE, MAVLINK_CRC_ATTITUDE,
           payload, sizeof(payload));
}

static void send_pressure(MavlinkTelemetry *telemetry,
                          const DataAcquisitionData *data)
{
  uint8_t payload[14] = {0};
  put_u32(payload, 0U, data->timestamp_ms);
  put_float(payload, 4U, value_or_zero(data->barometer_valid,
                                       data->barometer.pressure_pa * 0.01F));
  put_float(payload, 8U, value_or_zero(data->airspeed_valid,
                                       data->differential_pressure_pa * 0.01F));
  const int16_t temperature = data->barometer_valid ?
      (int16_t)lroundf(data->barometer.temperature_c * 100.0F) : 0;
  put_u16(payload, 12U, (uint16_t)temperature);
  transmit(telemetry, MAVLINK_MSG_ID_SCALED_PRESSURE,
           MAVLINK_CRC_SCALED_PRESSURE, payload, sizeof(payload));
}

static void send_gps(MavlinkTelemetry *telemetry,
                     const DataAcquisitionData *data)
{
  uint8_t payload[30] = {0};
  const bool valid = data->gps_valid;
  const bool position_valid = valid && data->gps.coordinates.valid;
  put_u64(payload, 0U, (uint64_t)data->timestamp_ms * 1000ULL);
  put_u32(payload, 8U, (uint32_t)scaled_i32(
      position_valid ? data->gps.coordinates.latitude_deg : 0.0, 1.0e7));
  put_u32(payload, 12U, (uint32_t)scaled_i32(
      position_valid ? data->gps.coordinates.longitude_deg : 0.0, 1.0e7));
  put_u32(payload, 16U, (uint32_t)scaled_i32(
      (valid && data->gps.altitude_valid) ? data->gps.altitude_m : 0.0, 1000.0));
  put_u16(payload, 20U, (valid && data->gps.dilution_valid) ?
      scaled_u16(data->gps.horizontal_dilution, 100.0F, UINT16_MAX) : UINT16_MAX);
  put_u16(payload, 22U, (valid && data->gps.dilution_valid) ?
      scaled_u16(data->gps.vertical_dilution, 100.0F, UINT16_MAX) : UINT16_MAX);
  put_u16(payload, 24U, (valid && data->gps.speed_valid) ?
      scaled_u16(data->gps.speed_mps, 100.0F, UINT16_MAX) : UINT16_MAX);
  put_u16(payload, 26U, (valid && data->gps.course_valid) ?
      scaled_u16(data->gps.course_deg, 100.0F, UINT16_MAX) : UINT16_MAX);
  payload[28] = position_valid ? (uint8_t)data->gps.fix_dimension : 1U;
  payload[29] = valid ? data->gps.satellites : 0U;
  transmit(telemetry, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_CRC_GPS_RAW_INT,
           payload, sizeof(payload));
}

static void send_hud(MavlinkTelemetry *telemetry,
                     const DataAcquisitionData *data)
{
  uint8_t payload[20] = {0};
  put_float(payload, 0U, value_or_zero(data->airspeed_valid,
                                       data->indicated_airspeed_mps));
  put_float(payload, 4U, (data->gps_valid && data->gps.speed_valid) ?
                          data->gps.speed_mps : 0.0F);
  put_float(payload, 8U, value_or_zero(data->barometer_valid,
                                       data->altitude_m));
  put_float(payload, 12U, 0.0F);
  float heading = value_or_zero(data->orientation_valid,
                                data->orientation.heading_deg);
  heading = fmodf(heading, 360.0F);
  if (heading < 0.0F) heading += 360.0F;
  put_u16(payload, 16U, (uint16_t)lroundf(heading) % 360U);
  put_u16(payload, 18U, 0U);
  transmit(telemetry, MAVLINK_MSG_ID_VFR_HUD, MAVLINK_CRC_VFR_HUD,
           payload, sizeof(payload));
}

void MavlinkTelemetry_Init(MavlinkTelemetry *telemetry,
                           UART_HandleTypeDef *uart, uint32_t now_ms)
{
  if (telemetry == NULL) return;
  memset(telemetry, 0, sizeof(*telemetry));
  telemetry->uart = uart;
  telemetry->last_heartbeat_ms = now_ms - HEARTBEAT_PERIOD_MS;
  telemetry->last_imu_ms = now_ms - HIGHRES_IMU_PERIOD_MS;
  telemetry->last_attitude_ms = now_ms - ATTITUDE_PERIOD_MS;
  telemetry->last_pressure_ms = now_ms - PRESSURE_PERIOD_MS;
  telemetry->last_gps_ms = now_ms - GPS_PERIOD_MS;
  telemetry->last_hud_ms = now_ms - HUD_PERIOD_MS;
  telemetry->last_fault_report_ms = now_ms;
  MavlinkEncoder_Init(&telemetry->encoder, MAVLINK_SYSTEM_ID,
                      MAVLINK_COMPONENT_ID);
}

void MavlinkTelemetry_Run(MavlinkTelemetry *telemetry,
                          const DataAcquisitionData *data, uint32_t now_ms)
{
  if ((telemetry == NULL) || (telemetry->uart == NULL) || (data == NULL)) return;
  update_utc_clock(telemetry, data, now_ms);
  const bool repeat_faults = elapsed(now_ms, &telemetry->last_fault_report_ms,
                                     FAULT_REPEAT_PERIOD_MS);
  report_health_changes(telemetry, data, repeat_faults, now_ms);
  if (elapsed(now_ms, &telemetry->last_heartbeat_ms, HEARTBEAT_PERIOD_MS))
    send_heartbeat(telemetry);
  if (elapsed(now_ms, &telemetry->last_imu_ms, HIGHRES_IMU_PERIOD_MS))
    send_highres_imu(telemetry, data);
  if (elapsed(now_ms, &telemetry->last_attitude_ms, ATTITUDE_PERIOD_MS))
    send_attitude(telemetry, data);
  if (elapsed(now_ms, &telemetry->last_pressure_ms, PRESSURE_PERIOD_MS))
    send_pressure(telemetry, data);
  if (elapsed(now_ms, &telemetry->last_gps_ms, GPS_PERIOD_MS))
    send_gps(telemetry, data);
  if (elapsed(now_ms, &telemetry->last_hud_ms, HUD_PERIOD_MS))
    send_hud(telemetry, data);
}
