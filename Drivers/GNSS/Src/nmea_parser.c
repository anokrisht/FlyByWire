#include "nmea_parser.h"

#include <stddef.h>
#include <string.h>

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

bool NmeaParser_Init(NmeaParser *parser, const ByteStream *stream)
{
  if ((parser == NULL) || (stream == NULL) || (stream->read_byte == NULL))
  {
    return false;
  }
  memset(parser, 0, sizeof(*parser));
  parser->stream = *stream;
  return true;
}

bool NmeaParser_Poll(NmeaParser *parser)
{
  uint8_t byte;
  if (parser == NULL)
  {
    return false;
  }
  parser->sentence_available = false;

  while (parser->stream.read_byte(parser->stream.context, &byte))
  {
    if (byte == '$')
    {
      parser->working_length = 0U;
      parser->working_sentence[parser->working_length++] = (char)byte;
    }
    else if ((byte == '\r') || (byte == '\n'))
    {
      if (parser->working_length == 0U)
      {
        continue;
      }
      parser->working_sentence[parser->working_length] = '\0';
      if (checksum_is_valid(parser->working_sentence, parser->working_length))
      {
        memcpy(parser->sentence, parser->working_sentence,
               (size_t)parser->working_length + 1U);
        ++parser->valid_sentence_count;
        parser->sentence_available = true;
      }
      else
      {
        ++parser->checksum_error_count;
      }
      parser->working_length = 0U;
      if (parser->sentence_available)
      {
        return true;
      }
    }
    else if (parser->working_length != 0U)
    {
      if (parser->working_length < (NMEA_MAX_SENTENCE_LENGTH - 1U))
      {
        parser->working_sentence[parser->working_length++] = (char)byte;
      }
      else
      {
        parser->working_length = 0U;
        ++parser->framing_error_count;
      }
    }
  }
  return false;
}

const char *NmeaParser_GetSentence(const NmeaParser *parser)
{
  return ((parser != NULL) && parser->sentence_available)
             ? parser->sentence
             : NULL;
}
