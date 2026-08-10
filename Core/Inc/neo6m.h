#ifndef NEO6M_H
#define NEO6M_H

#include "byte_stream.h"

#include <stdbool.h>
#include <stdint.h>

#define NEO6M_MAX_SENTENCE_LENGTH 128U

typedef struct
{
  double latitude_deg;
  double longitude_deg;
  bool valid;
} Neo6m_Coordinates;

typedef struct
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;
  bool valid;
} Neo6m_Time;

typedef struct
{
  uint8_t day;
  uint8_t month;
  uint16_t year;
  bool valid;
} Neo6m_Date;

typedef struct
{
  Neo6m_Coordinates coordinates;
  Neo6m_Time utc_time;
  Neo6m_Date utc_date;
  float altitude_m;
  float speed_mps;
  float course_deg;
  float horizontal_dilution;
  uint8_t satellites;
  uint8_t fix_quality;
  bool altitude_valid;
  bool speed_valid;
  bool course_valid;
} Neo6m_Data;

typedef struct
{
  ByteStream stream;
  char working_sentence[NEO6M_MAX_SENTENCE_LENGTH];
  char sentence[NEO6M_MAX_SENTENCE_LENGTH];
  uint16_t working_length;
  uint32_t valid_sentence_count;
  uint32_t checksum_error_count;
  uint32_t framing_error_count;
  bool sentence_available;
  Neo6m_Data data;
} Neo6m;

bool Neo6m_Init(Neo6m *gps, const ByteStream *stream);

/** Consume pending input. Returns true when a verified NMEA sentence is ready. */
bool Neo6m_Poll(Neo6m *gps);

/** Return the most recently completed sentence, including '$' but no CR/LF. */
const char *Neo6m_GetSentence(const Neo6m *gps);

/** Latest structured values assembled from GLL, GGA, and RMC sentences. */
const Neo6m_Data *Neo6m_GetData(const Neo6m *gps);
bool Neo6m_GetCoordinates(const Neo6m *gps, Neo6m_Coordinates *coordinates);
bool Neo6m_GetUtcTime(const Neo6m *gps, Neo6m_Time *time);
bool Neo6m_GetUtcDate(const Neo6m *gps, Neo6m_Date *date);

/**
 * Convert UTC to a fixed-offset local time. For example, use 60 for UTC+1.
 * day_offset is set to -1, 0, or +1 when the conversion crosses midnight.
 */
bool Neo6m_GetLocalTime(const Neo6m *gps, int16_t utc_offset_minutes,
                        Neo6m_Time *time, int8_t *day_offset);

#endif /* NEO6M_H */
