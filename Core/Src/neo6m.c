#include "neo6m.h"

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
  double fraction_scale = 0.1;
  bool fractional = false;
  bool negative = false;
  uint16_t first_digit = 0U;
  if (field.text[0] == '-')
  {
    negative = true;
    first_digit = 1U;
  }
  if (first_digit == field.length)
  {
    return false;
  }
  for (uint16_t index = first_digit; index < field.length; ++index)
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
        result += (double)digit * fraction_scale;
        fraction_scale *= 0.1;
      }
      else
      {
        result = result * 10.0 + (double)digit;
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

static bool parse_time(NmeaField field, Neo6m_Time *time)
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

static void parse_position(Neo6m *gps, uint8_t latitude_field)
{
  double latitude;
  double longitude;
  const bool valid =
      parse_coordinate(get_field(gps->sentence, latitude_field),
                       get_field(gps->sentence, latitude_field + 1U), 2U,
                       &latitude) &&
      parse_coordinate(get_field(gps->sentence, latitude_field + 2U),
                       get_field(gps->sentence, latitude_field + 3U), 3U,
                       &longitude);
  if (valid)
  {
    gps->data.coordinates.latitude_deg = latitude;
    gps->data.coordinates.longitude_deg = longitude;
  }
  gps->data.coordinates.valid = valid;
}

static void parse_sentence(Neo6m *gps)
{
  if (strlen(gps->sentence) < 6U)
  {
    return;
  }
  const char *type = &gps->sentence[3];
  double value;
  uint32_t integer;

  if (strncmp(type, "GLL", 3U) == 0)
  {
    parse_position(gps, 1U);
    (void)parse_time(get_field(gps->sentence, 5U), &gps->data.utc_time);
    const NmeaField status = get_field(gps->sentence, 6U);
    gps->data.coordinates.valid &=
        (status.length == 1U) && (status.text[0] == 'A');
  }
  else if (strncmp(type, "GGA", 3U) == 0)
  {
    (void)parse_time(get_field(gps->sentence, 1U), &gps->data.utc_time);
    parse_position(gps, 2U);
    if (parse_unsigned(get_field(gps->sentence, 6U), &integer))
    {
      gps->data.fix_quality = (uint8_t)integer;
      gps->data.coordinates.valid &= integer != 0U;
    }
    if (parse_unsigned(get_field(gps->sentence, 7U), &integer))
    {
      gps->data.satellites = (uint8_t)integer;
    }
    if (parse_decimal(get_field(gps->sentence, 8U), &value))
    {
      gps->data.horizontal_dilution = (float)value;
    }
    gps->data.altitude_valid =
        parse_decimal(get_field(gps->sentence, 9U), &value);
    if (gps->data.altitude_valid)
    {
      gps->data.altitude_m = (float)value;
    }
  }
  else if (strncmp(type, "RMC", 3U) == 0)
  {
    (void)parse_time(get_field(gps->sentence, 1U), &gps->data.utc_time);
    parse_position(gps, 3U);
    const NmeaField status = get_field(gps->sentence, 2U);
    gps->data.coordinates.valid &=
        (status.length == 1U) && (status.text[0] == 'A');
    gps->data.speed_valid = parse_decimal(get_field(gps->sentence, 7U), &value);
    if (gps->data.speed_valid)
    {
      gps->data.speed_mps = (float)(value * 0.514444444);
    }
    gps->data.course_valid = parse_decimal(get_field(gps->sentence, 8U), &value);
    if (gps->data.course_valid)
    {
      gps->data.course_deg = (float)value;
    }
    const NmeaField date = get_field(gps->sentence, 9U);
    if ((date.length == 6U) && parse_unsigned(date, &integer))
    {
      gps->data.utc_date.day = (uint8_t)(integer / 10000U);
      gps->data.utc_date.month = (uint8_t)((integer / 100U) % 100U);
      const uint16_t short_year = (uint16_t)(integer % 100U);
      gps->data.utc_date.year =
          (uint16_t)((short_year >= 80U ? 1900U : 2000U) + short_year);
      gps->data.utc_date.valid = (gps->data.utc_date.day >= 1U) &&
                                 (gps->data.utc_date.day <= 31U) &&
                                 (gps->data.utc_date.month >= 1U) &&
                                 (gps->data.utc_date.month <= 12U);
    }
  }
}

static int8_t hex_value(char character)
{
  if ((character >= '0') && (character <= '9'))
  {
    return (int8_t)(character - '0');
  }
  if ((character >= 'A') && (character <= 'F'))
  {
    return (int8_t)(character - 'A' + 10);
  }
  if ((character >= 'a') && (character <= 'f'))
  {
    return (int8_t)(character - 'a' + 10);
  }
  return -1;
}

static bool checksum_is_valid(const char *sentence, uint16_t length)
{
  uint8_t checksum = 0U;
  uint16_t delimiter = 0U;

  if ((length < 5U) || (sentence[0] != '$'))
  {
    return false;
  }
  for (uint16_t index = 1U; index < length; ++index)
  {
    if (sentence[index] == '*')
    {
      delimiter = index;
      break;
    }
    checksum ^= (uint8_t)sentence[index];
  }
  if ((delimiter == 0U) || ((uint16_t)(delimiter + 3U) != length))
  {
    return false;
  }

  const int8_t high = hex_value(sentence[delimiter + 1U]);
  const int8_t low = hex_value(sentence[delimiter + 2U]);
  return (high >= 0) && (low >= 0) &&
         (checksum == (uint8_t)(((uint8_t)high << 4) | (uint8_t)low));
}

bool Neo6m_Init(Neo6m *gps, const ByteStream *stream)
{
  if ((gps == NULL) || (stream == NULL) || (stream->read_byte == NULL))
  {
    return false;
  }
  memset(gps, 0, sizeof(*gps));
  gps->stream = *stream;
  return true;
}

bool Neo6m_Poll(Neo6m *gps)
{
  uint8_t byte;
  if (gps == NULL)
  {
    return false;
  }
  gps->sentence_available = false;

  while (gps->stream.read_byte(gps->stream.context, &byte))
  {
    if (byte == '$')
    {
      gps->working_length = 0U;
      gps->working_sentence[gps->working_length++] = (char)byte;
    }
    else if ((byte == '\r') || (byte == '\n'))
    {
      if (gps->working_length == 0U)
      {
        continue;
      }
      gps->working_sentence[gps->working_length] = '\0';
      if (checksum_is_valid(gps->working_sentence, gps->working_length))
      {
        memcpy(gps->sentence, gps->working_sentence,
               (size_t)gps->working_length + 1U);
        parse_sentence(gps);
        ++gps->valid_sentence_count;
        gps->sentence_available = true;
      }
      else
      {
        ++gps->checksum_error_count;
      }
      gps->working_length = 0U;
      if (gps->sentence_available)
      {
        return true;
      }
    }
    else if (gps->working_length != 0U)
    {
      if (gps->working_length < (NEO6M_MAX_SENTENCE_LENGTH - 1U))
      {
        gps->working_sentence[gps->working_length++] = (char)byte;
      }
      else
      {
        gps->working_length = 0U;
        ++gps->framing_error_count;
      }
    }
  }
  return false;
}

const char *Neo6m_GetSentence(const Neo6m *gps)
{
  return ((gps != NULL) && gps->sentence_available) ? gps->sentence : NULL;
}

const Neo6m_Data *Neo6m_GetData(const Neo6m *gps)
{
  return (gps != NULL) ? &gps->data : NULL;
}

bool Neo6m_GetCoordinates(const Neo6m *gps, Neo6m_Coordinates *coordinates)
{
  if ((gps == NULL) || (coordinates == NULL) || !gps->data.coordinates.valid)
  {
    return false;
  }
  *coordinates = gps->data.coordinates;
  return true;
}

bool Neo6m_GetUtcTime(const Neo6m *gps, Neo6m_Time *time)
{
  if ((gps == NULL) || (time == NULL) || !gps->data.utc_time.valid)
  {
    return false;
  }
  *time = gps->data.utc_time;
  return true;
}

bool Neo6m_GetUtcDate(const Neo6m *gps, Neo6m_Date *date)
{
  if ((gps == NULL) || (date == NULL) || !gps->data.utc_date.valid)
  {
    return false;
  }
  *date = gps->data.utc_date;
  return true;
}

bool Neo6m_GetLocalTime(const Neo6m *gps, int16_t utc_offset_minutes,
                        Neo6m_Time *time, int8_t *day_offset)
{
  if ((gps == NULL) || (time == NULL) || (day_offset == NULL) ||
      !gps->data.utc_time.valid || (utc_offset_minutes < -1439) ||
      (utc_offset_minutes > 1439))
  {
    return false;
  }

  int32_t minutes = (int32_t)gps->data.utc_time.hours * 60 +
                    gps->data.utc_time.minutes + utc_offset_minutes;
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
  *time = gps->data.utc_time;
  time->hours = (uint8_t)(minutes / 60);
  time->minutes = (uint8_t)(minutes % 60);
  return true;
}
