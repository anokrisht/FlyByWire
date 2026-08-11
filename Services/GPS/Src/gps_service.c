#include "gps_service.h"

#include <stddef.h>
#include <string.h>

typedef struct
{
  const char *text;
  uint16_t length;
} NmeaField;

static NmeaField get_field(const char *sentence, uint8_t requested)
{
  NmeaField field = {0};
  uint8_t current = 0U;
  const char *start = sentence;

  for (const char *cursor = sentence; ; ++cursor)
  {
    if ((*cursor == ',') || (*cursor == '*') || (*cursor == '\0'))
    {
      if (current == requested)
      {
        field.text = start;
        field.length = (uint16_t)(cursor - start);
        return field;
      }
      if ((*cursor == '*') || (*cursor == '\0'))
      {
        return field;
      }
      ++current;
      start = cursor + 1;
    }
  }
}

static bool parse_unsigned(NmeaField field, uint32_t *value)
{
  if ((field.length == 0U) || (value == NULL))
  {
    return false;
  }
  uint32_t result = 0U;
  for (uint16_t index = 0U; index < field.length; ++index)
  {
    if ((field.text[index] < '0') || (field.text[index] > '9'))
    {
      return false;
    }
    result = result * 10U + (uint32_t)(field.text[index] - '0');
  }
  *value = result;
  return true;
}

static bool parse_decimal(NmeaField field, double *value)
{
  if ((field.length == 0U) || (value == NULL))
  {
    return false;
  }
  double result = 0.0;
  double scale = 0.1;
  bool fractional = false;
  bool negative = false;
  uint16_t index = 0U;
  if (field.text[0] == '-')
  {
    negative = true;
    index = 1U;
  }
  if (index == field.length)
  {
    return false;
  }
  for (; index < field.length; ++index)
  {
    const char character = field.text[index];
    if (character == '.')
    {
      if (fractional)
      {
        return false;
      }
      fractional = true;
    }
    else if ((character >= '0') && (character <= '9'))
    {
      const uint8_t digit = (uint8_t)(character - '0');
      if (fractional)
      {
        result += (double)digit * scale;
        scale *= 0.1;
      }
      else
      {
        result = result * 10.0 + digit;
      }
    }
    else
    {
      return false;
    }
  }
  *value = negative ? -result : result;
  return true;
}

static bool parse_time(NmeaField field, GpsTime *time)
{
  double raw;
  if ((field.length < 6U) || !parse_decimal(field, &raw))
  {
    return false;
  }
  const uint32_t whole = (uint32_t)raw;
  time->hours = (uint8_t)(whole / 10000U);
  time->minutes = (uint8_t)((whole / 100U) % 100U);
  time->seconds = (uint8_t)(whole % 100U);
  time->milliseconds = (uint16_t)((raw - (double)whole) * 1000.0 + 0.5);
  time->valid = (time->hours < 24U) && (time->minutes < 60U) &&
                (time->seconds < 60U);
  return time->valid;
}

static bool parse_coordinate(NmeaField coordinate, NmeaField hemisphere,
                             uint8_t degree_digits, double *degrees)
{
  double raw;
  if ((coordinate.length <= degree_digits) || (hemisphere.length != 1U) ||
      !parse_decimal(coordinate, &raw))
  {
    return false;
  }
  const uint32_t whole_degrees = (uint32_t)(raw / 100.0);
  double result = (double)whole_degrees +
                  (raw - (double)whole_degrees * 100.0) / 60.0;
  const char direction = hemisphere.text[0];
  if ((direction == 'S') || (direction == 'W'))
  {
    result = -result;
  }
  else if ((direction != 'N') && (direction != 'E'))
  {
    return false;
  }
  *degrees = result;
  return true;
}

static char field_character(const char *sentence, uint8_t index)
{
  const NmeaField field = get_field(sentence, index);
  return (field.length == 1U) ? field.text[0] : '\0';
}

static GpsConstellation constellation_from_talker(const char *sentence)
{
  if ((sentence == NULL) || (sentence[0] != '$'))
  {
    return GPS_CONSTELLATION_UNKNOWN;
  }
  if ((sentence[1] == 'G') && (sentence[2] == 'P'))
  {
    return GPS_CONSTELLATION_GPS;
  }
  if ((sentence[1] == 'G') && (sentence[2] == 'L'))
  {
    return GPS_CONSTELLATION_GLONASS;
  }
  if ((sentence[1] == 'G') && (sentence[2] == 'A'))
  {
    return GPS_CONSTELLATION_GALILEO;
  }
  if (((sentence[1] == 'G') && (sentence[2] == 'B')) ||
      ((sentence[1] == 'B') && (sentence[2] == 'D')))
  {
    return GPS_CONSTELLATION_BEIDOU;
  }
  if ((sentence[1] == 'G') && (sentence[2] == 'Q'))
  {
    return GPS_CONSTELLATION_QZSS;
  }
  if ((sentence[1] == 'G') && (sentence[2] == 'N'))
  {
    return GPS_CONSTELLATION_MIXED;
  }
  return GPS_CONSTELLATION_UNKNOWN;
}

static GpsConstellation constellation_from_system_id(uint32_t system_id)
{
  switch (system_id)
  {
    case 1U: return GPS_CONSTELLATION_GPS;
    case 2U: return GPS_CONSTELLATION_GLONASS;
    case 3U: return GPS_CONSTELLATION_GALILEO;
    case 4U: return GPS_CONSTELLATION_BEIDOU;
    case 5U: return GPS_CONSTELLATION_QZSS;
    default: return GPS_CONSTELLATION_UNKNOWN;
  }
}

static void remove_visible_constellation(GpsData *data,
                                         GpsConstellation constellation)
{
  uint8_t destination = 0U;
  for (uint8_t index = 0U; index < data->visible_satellite_count; ++index)
  {
    if (data->visible_satellites[index].constellation != constellation)
    {
      data->visible_satellites[destination++] = data->visible_satellites[index];
    }
  }
  data->visible_satellite_count = destination;
}

static void remove_used_constellation(GpsData *data,
                                      GpsConstellation constellation)
{
  uint8_t destination = 0U;
  for (uint8_t index = 0U; index < data->used_satellite_count; ++index)
  {
    if (data->used_satellites[index].constellation != constellation)
    {
      data->used_satellites[destination++] = data->used_satellites[index];
    }
  }
  data->used_satellite_count = destination;
}

static void parse_position(GpsService *service, const char *sentence,
                           uint8_t latitude_field)
{
  double latitude;
  double longitude;
  const bool valid =
      parse_coordinate(get_field(sentence, latitude_field),
                       get_field(sentence, latitude_field + 1U), 2U,
                       &latitude) &&
      parse_coordinate(get_field(sentence, latitude_field + 2U),
                       get_field(sentence, latitude_field + 3U), 3U,
                       &longitude);
  if (valid)
  {
    service->data.coordinates.latitude_deg = latitude;
    service->data.coordinates.longitude_deg = longitude;
  }
  service->data.coordinates.valid = valid;
}

static void parse_gll(GpsService *service, const char *sentence)
{
  parse_position(service, sentence, 1U);
  (void)parse_time(get_field(sentence, 5U), &service->data.utc_time);
  const NmeaField status = get_field(sentence, 6U);
  service->data.coordinates.valid &=
      (status.length == 1U) && (status.text[0] == 'A');
  service->data.positioning_mode = field_character(sentence, 7U);
}

static void parse_gga(GpsService *service, const char *sentence)
{
  double value;
  uint32_t integer;
  (void)parse_time(get_field(sentence, 1U), &service->data.utc_time);
  parse_position(service, sentence, 2U);
  if (parse_unsigned(get_field(sentence, 6U), &integer))
  {
    service->data.fix_quality = (uint8_t)integer;
    service->data.coordinates.valid &= integer != 0U;
  }
  if (parse_unsigned(get_field(sentence, 7U), &integer))
  {
    service->data.satellites = (uint8_t)integer;
  }
  if (parse_decimal(get_field(sentence, 8U), &value))
  {
    service->data.horizontal_dilution = (float)value;
  }
  service->data.altitude_valid =
      parse_decimal(get_field(sentence, 9U), &value);
  if (service->data.altitude_valid)
  {
    service->data.altitude_m = (float)value;
  }
  service->data.geoid_separation_valid =
      parse_decimal(get_field(sentence, 11U), &value);
  if (service->data.geoid_separation_valid)
  {
    service->data.geoid_separation_m = (float)value;
  }
  service->data.differential_age_valid =
      parse_decimal(get_field(sentence, 13U), &value);
  if (service->data.differential_age_valid)
  {
    service->data.differential_age_s = (float)value;
  }
  service->data.differential_station_valid =
      parse_unsigned(get_field(sentence, 14U), &integer);
  if (service->data.differential_station_valid)
  {
    service->data.differential_station_id = (uint16_t)integer;
  }
}

static void parse_rmc(GpsService *service, const char *sentence)
{
  double value;
  uint32_t integer;
  (void)parse_time(get_field(sentence, 1U), &service->data.utc_time);
  parse_position(service, sentence, 3U);
  const NmeaField status = get_field(sentence, 2U);
  service->data.coordinates.valid &=
      (status.length == 1U) && (status.text[0] == 'A');
  service->data.speed_valid = parse_decimal(get_field(sentence, 7U), &value);
  if (service->data.speed_valid)
  {
    service->data.speed_knots = (float)value;
    service->data.speed_mps = (float)(value * 0.514444444);
    service->data.speed_kph = (float)(value * 1.852);
  }
  service->data.course_valid = parse_decimal(get_field(sentence, 8U), &value);
  if (service->data.course_valid)
  {
    service->data.course_deg = (float)value;
  }
  service->data.magnetic_variation_valid =
      parse_decimal(get_field(sentence, 10U), &value);
  if (service->data.magnetic_variation_valid)
  {
    if (field_character(sentence, 11U) == 'W')
    {
      value = -value;
    }
    service->data.magnetic_variation_deg = (float)value;
  }
  const NmeaField date = get_field(sentence, 9U);
  if ((date.length == 6U) && parse_unsigned(date, &integer))
  {
    service->data.utc_date.day = (uint8_t)(integer / 10000U);
    service->data.utc_date.month = (uint8_t)((integer / 100U) % 100U);
    const uint16_t short_year = (uint16_t)(integer % 100U);
    service->data.utc_date.year =
        (uint16_t)((short_year >= 80U ? 1900U : 2000U) + short_year);
      service->data.utc_date.valid =
        (service->data.utc_date.day >= 1U) &&
        (service->data.utc_date.day <= 31U) &&
        (service->data.utc_date.month >= 1U) &&
        (service->data.utc_date.month <= 12U);
  }
  service->data.positioning_mode = field_character(sentence, 12U);
  service->data.navigation_status = field_character(sentence, 13U);
}

static void parse_vtg(GpsService *service, const char *sentence)
{
  double value;
  service->data.course_valid = parse_decimal(get_field(sentence, 1U), &value);
  if (service->data.course_valid)
  {
    service->data.course_deg = (float)value;
  }
  service->data.magnetic_course_valid =
      parse_decimal(get_field(sentence, 3U), &value);
  if (service->data.magnetic_course_valid)
  {
    service->data.magnetic_course_deg = (float)value;
  }
  service->data.speed_valid = parse_decimal(get_field(sentence, 5U), &value);
  if (service->data.speed_valid)
  {
    service->data.speed_knots = (float)value;
    service->data.speed_mps = (float)(value * 0.514444444);
    if (parse_decimal(get_field(sentence, 7U), &value))
    {
      service->data.speed_kph = (float)value;
    }
    else
    {
      service->data.speed_kph = service->data.speed_knots * 1.852F;
    }
  }
  service->data.positioning_mode = field_character(sentence, 9U);
}

static void parse_gsa(GpsService *service, const char *sentence)
{
  double value;
  uint32_t integer;
  GpsConstellation constellation = constellation_from_talker(sentence);
  if (parse_unsigned(get_field(sentence, 18U), &integer))
  {
    constellation = constellation_from_system_id(integer);
  }
  service->data.active_constellation = constellation;
  service->data.selection_mode = field_character(sentence, 1U);
  if (parse_unsigned(get_field(sentence, 2U), &integer) &&
      (integer >= GPS_FIX_NONE) && (integer <= GPS_FIX_3D))
  {
    service->data.fix_dimension = (GpsFixDimension)integer;
  }

  remove_used_constellation(&service->data, constellation);
  for (uint8_t field_index = 3U; field_index <= 14U; ++field_index)
  {
    if (parse_unsigned(get_field(sentence, field_index), &integer) &&
        (service->data.used_satellite_count < GPS_MAX_USED_SATELLITES))
    {
      GpsUsedSatellite *satellite =
          &service->data.used_satellites[service->data.used_satellite_count++];
      satellite->id = (uint16_t)integer;
      satellite->constellation = constellation;
    }
  }
  const bool pdop_valid = parse_decimal(get_field(sentence, 15U), &value);
  if (pdop_valid)
  {
    service->data.position_dilution = (float)value;
  }
  const bool hdop_valid = parse_decimal(get_field(sentence, 16U), &value);
  if (hdop_valid)
  {
    service->data.horizontal_dilution = (float)value;
  }
  const bool vdop_valid = parse_decimal(get_field(sentence, 17U), &value);
  if (vdop_valid)
  {
    service->data.vertical_dilution = (float)value;
  }
  service->data.dilution_valid = pdop_valid && hdop_valid && vdop_valid;
}

static void parse_gsv(GpsService *service, const char *sentence)
{
  uint32_t integer;
  const GpsConstellation constellation = constellation_from_talker(sentence);
  if (parse_unsigned(get_field(sentence, 2U), &integer) && (integer == 1U))
  {
    remove_visible_constellation(&service->data, constellation);
  }

  for (uint8_t group = 0U; group < 4U; ++group)
  {
    const uint8_t base = (uint8_t)(4U + group * 4U);
    if (!parse_unsigned(get_field(sentence, base), &integer) ||
        (service->data.visible_satellite_count >= GPS_MAX_VISIBLE_SATELLITES))
    {
      continue;
    }
    GpsSatellite *satellite =
        &service->data.visible_satellites[service->data.visible_satellite_count++];
    memset(satellite, 0, sizeof(*satellite));
    satellite->id = (uint16_t)integer;
    satellite->constellation = constellation;
    satellite->elevation_valid =
        parse_unsigned(get_field(sentence, base + 1U), &integer);
    if (satellite->elevation_valid)
    {
      satellite->elevation_deg = (int16_t)integer;
    }
    satellite->azimuth_valid =
        parse_unsigned(get_field(sentence, base + 2U), &integer);
    if (satellite->azimuth_valid)
    {
      satellite->azimuth_deg = (uint16_t)integer;
    }
    satellite->signal_strength_valid =
        parse_unsigned(get_field(sentence, base + 3U), &integer);
    if (satellite->signal_strength_valid)
    {
      satellite->signal_strength_dbhz = (uint8_t)integer;
    }
  }
  service->data.satellites_in_view = service->data.visible_satellite_count;
}

bool GpsService_Init(GpsService *service, const ByteStream *stream)
{
  if (service == NULL)
  {
    return false;
  }
  memset(service, 0, sizeof(*service));
  return NmeaParser_Init(&service->parser, stream);
}

bool GpsService_Update(GpsService *service)
{
  if ((service == NULL) || !NmeaParser_Poll(&service->parser))
  {
    return false;
  }
  const char *sentence = NmeaParser_GetSentence(&service->parser);
  if ((sentence == NULL) || (strlen(sentence) < 6U))
  {
    return true;
  }
  const char *type = &sentence[3];
  if (strncmp(type, "GLL", 3U) == 0)
  {
    parse_gll(service, sentence);
  }
  else if (strncmp(type, "GGA", 3U) == 0)
  {
    parse_gga(service, sentence);
  }
  else if (strncmp(type, "RMC", 3U) == 0)
  {
    parse_rmc(service, sentence);
  }
  else if (strncmp(type, "VTG", 3U) == 0)
  {
    parse_vtg(service, sentence);
  }
  else if (strncmp(type, "GSA", 3U) == 0)
  {
    parse_gsa(service, sentence);
  }
  else if (strncmp(type, "GSV", 3U) == 0)
  {
    parse_gsv(service, sentence);
  }
  return true;
}

const char *GpsService_GetRawSentence(const GpsService *service)
{
  return (service != NULL) ? NmeaParser_GetSentence(&service->parser) : NULL;
}

const GpsData *GpsService_GetData(const GpsService *service)
{
  return (service != NULL) ? &service->data : NULL;
}

bool GpsService_GetDiagnostics(const GpsService *service,
                               GpsDiagnostics *diagnostics)
{
  if ((service == NULL) || (diagnostics == NULL))
  {
    return false;
  }
  diagnostics->valid_sentence_count = service->parser.valid_sentence_count;
  diagnostics->checksum_error_count = service->parser.checksum_error_count;
  diagnostics->framing_error_count = service->parser.framing_error_count;
  return true;
}

bool GpsService_GetCoordinates(const GpsService *service,
                               GpsCoordinates *coordinates)
{
  if ((service == NULL) || (coordinates == NULL) ||
      !service->data.coordinates.valid)
  {
    return false;
  }
  *coordinates = service->data.coordinates;
  return true;
}

bool GpsService_GetUtcTime(const GpsService *service, GpsTime *time)
{
  if ((service == NULL) || (time == NULL) || !service->data.utc_time.valid)
  {
    return false;
  }
  *time = service->data.utc_time;
  return true;
}

bool GpsService_GetUtcDate(const GpsService *service, GpsDate *date)
{
  if ((service == NULL) || (date == NULL) || !service->data.utc_date.valid)
  {
    return false;
  }
  *date = service->data.utc_date;
  return true;
}

bool GpsService_GetLocalTime(const GpsService *service,
                             int16_t utc_offset_minutes, GpsTime *time,
                             int8_t *day_offset)
{
  if ((service == NULL) || (time == NULL) || (day_offset == NULL) ||
      !service->data.utc_time.valid || (utc_offset_minutes < -1439) ||
      (utc_offset_minutes > 1439))
  {
    return false;
  }
  int32_t minutes = (int32_t)service->data.utc_time.hours * 60 +
                    service->data.utc_time.minutes + utc_offset_minutes;
  *day_offset = 0;
  if (minutes < 0)
  {
    minutes += 1440;
    *day_offset = -1;
  }
  else if (minutes >= 1440)
  {
    minutes -= 1440;
    *day_offset = 1;
  }
  *time = service->data.utc_time;
  time->hours = (uint8_t)(minutes / 60);
  time->minutes = (uint8_t)(minutes % 60);
  return true;
}

void GpsService_Invalidate(GpsService *service)
{
  if (service != NULL)
  {
    service->data.coordinates.valid = false;
    service->data.utc_time.valid = false;
    service->data.utc_date.valid = false;
    service->data.altitude_valid = false;
    service->data.speed_valid = false;
    service->data.course_valid = false;
    service->data.magnetic_course_valid = false;
    service->data.magnetic_variation_valid = false;
    service->data.dilution_valid = false;
    service->data.geoid_separation_valid = false;
    service->data.differential_age_valid = false;
    service->data.differential_station_valid = false;
    service->data.fix_quality = 0U;
    service->data.fix_dimension = GPS_FIX_NONE;
    service->data.satellites = 0U;
    service->data.satellites_in_view = 0U;
    service->data.used_satellite_count = 0U;
    service->data.visible_satellite_count = 0U;
    service->data.positioning_mode = '\0';
    service->data.navigation_status = '\0';
    service->data.selection_mode = '\0';
    service->data.active_constellation = GPS_CONSTELLATION_UNKNOWN;
  }
}
