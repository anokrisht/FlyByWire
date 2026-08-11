#ifndef GPS_SERVICE_H
#define GPS_SERVICE_H

#include "nmea_parser.h"

#include <stdbool.h>
#include <stdint.h>

#define GPS_MAX_VISIBLE_SATELLITES 64U
#define GPS_MAX_USED_SATELLITES    24U

typedef enum
{
  GPS_CONSTELLATION_UNKNOWN = 0,
  GPS_CONSTELLATION_GPS,
  GPS_CONSTELLATION_GLONASS,
  GPS_CONSTELLATION_GALILEO,
  GPS_CONSTELLATION_BEIDOU,
  GPS_CONSTELLATION_QZSS,
  GPS_CONSTELLATION_MIXED
} GpsConstellation;

typedef enum
{
  GPS_FIX_NONE = 1,
  GPS_FIX_2D = 2,
  GPS_FIX_3D = 3
} GpsFixDimension;

typedef struct
{
  uint16_t id;
  int16_t elevation_deg;
  uint16_t azimuth_deg;
  uint8_t signal_strength_dbhz;
  GpsConstellation constellation;
  bool elevation_valid;
  bool azimuth_valid;
  bool signal_strength_valid;
} GpsSatellite;

typedef struct
{
  uint16_t id;
  GpsConstellation constellation;
} GpsUsedSatellite;

typedef struct
{
  double latitude_deg;
  double longitude_deg;
  bool valid;
} GpsCoordinates;

typedef struct
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;
  bool valid;
} GpsTime;

typedef struct
{
  uint8_t day;
  uint8_t month;
  uint16_t year;
  bool valid;
} GpsDate;

typedef struct
{
  GpsCoordinates coordinates;
  GpsTime utc_time;
  GpsDate utc_date;
  float altitude_m;
  float speed_mps;
  float speed_knots;
  float speed_kph;
  float course_deg;
  float magnetic_course_deg;
  float magnetic_variation_deg;
  float horizontal_dilution;
  float vertical_dilution;
  float position_dilution;
  float geoid_separation_m;
  float differential_age_s;
  uint8_t satellites;
  uint8_t satellites_in_view;
  uint8_t fix_quality;
  GpsFixDimension fix_dimension;
  GpsConstellation active_constellation;
  uint16_t differential_station_id;
  GpsUsedSatellite used_satellites[GPS_MAX_USED_SATELLITES];
  uint8_t used_satellite_count;
  GpsSatellite visible_satellites[GPS_MAX_VISIBLE_SATELLITES];
  uint8_t visible_satellite_count;
  char positioning_mode;
  char navigation_status;
  char selection_mode;
  bool altitude_valid;
  bool speed_valid;
  bool course_valid;
  bool magnetic_course_valid;
  bool magnetic_variation_valid;
  bool dilution_valid;
  bool geoid_separation_valid;
  bool differential_age_valid;
  bool differential_station_valid;
} GpsData;

typedef struct
{
  uint32_t valid_sentence_count;
  uint32_t checksum_error_count;
  uint32_t framing_error_count;
} GpsDiagnostics;

typedef struct
{
  NmeaParser parser;
  GpsData data;
} GpsService;

bool GpsService_Init(GpsService *service, const ByteStream *stream);
bool GpsService_Update(GpsService *service);
const char *GpsService_GetRawSentence(const GpsService *service);
const GpsData *GpsService_GetData(const GpsService *service);
bool GpsService_GetDiagnostics(const GpsService *service,
                               GpsDiagnostics *diagnostics);
bool GpsService_GetCoordinates(const GpsService *service,
                               GpsCoordinates *coordinates);
bool GpsService_GetUtcTime(const GpsService *service, GpsTime *time);
bool GpsService_GetUtcDate(const GpsService *service, GpsDate *date);
bool GpsService_GetLocalTime(const GpsService *service,
                             int16_t utc_offset_minutes, GpsTime *time,
                             int8_t *day_offset);
void GpsService_Invalidate(GpsService *service);

#endif /* GPS_SERVICE_H */
