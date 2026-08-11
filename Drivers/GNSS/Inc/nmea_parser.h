#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include "byte_stream.h"

#include <stdbool.h>
#include <stdint.h>

#define NMEA_MAX_SENTENCE_LENGTH 128U

typedef struct
{
  ByteStream stream;
  char working_sentence[NMEA_MAX_SENTENCE_LENGTH];
  char sentence[NMEA_MAX_SENTENCE_LENGTH];
  uint16_t working_length;
  uint32_t valid_sentence_count;
  uint32_t checksum_error_count;
  uint32_t framing_error_count;
  bool sentence_available;
} NmeaParser;

bool NmeaParser_Init(NmeaParser *parser, const ByteStream *stream);

/** Consume input until one verified sentence is available or input is empty. */
bool NmeaParser_Poll(NmeaParser *parser);

/** Latest verified sentence, including '$' but excluding CR/LF. */
const char *NmeaParser_GetSentence(const NmeaParser *parser);

#endif /* NMEA_PARSER_H */
