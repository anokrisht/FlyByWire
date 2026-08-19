#include "mavlink_encoder.h"

#include <stddef.h>

#define MAVLINK2_MAGIC 0xFDU
#define MAVLINK2_HEADER_LENGTH 10U
#define MAVLINK2_CHECKSUM_LENGTH 2U

static void crc_accumulate(uint8_t byte, uint16_t *crc)
{
  uint8_t temporary = byte ^ (uint8_t)(*crc & 0xFFU);
  temporary ^= (uint8_t)(temporary << 4U);
  *crc = (uint16_t)((*crc >> 8U) ^ ((uint16_t)temporary << 8U) ^
                    ((uint16_t)temporary << 3U) ^
                    ((uint16_t)temporary >> 4U));
}

void MavlinkEncoder_Init(MavlinkEncoder *encoder, uint8_t system_id,
                         uint8_t component_id)
{
  if (encoder == NULL)
  {
    return;
  }
  encoder->system_id = system_id;
  encoder->component_id = component_id;
  encoder->sequence = 0U;
}

size_t MavlinkEncoder_Encode(MavlinkEncoder *encoder, uint32_t message_id,
                             uint8_t crc_extra, const uint8_t *payload,
                             uint8_t payload_length, uint8_t *packet,
                             size_t packet_capacity)
{
  const size_t packet_length = MAVLINK2_HEADER_LENGTH + payload_length +
                               MAVLINK2_CHECKSUM_LENGTH;
  if ((encoder == NULL) || (packet == NULL) ||
      ((payload == NULL) && (payload_length != 0U)) ||
      (message_id > 0xFFFFFFUL) || (packet_capacity < packet_length))
  {
    return 0U;
  }

  packet[0] = MAVLINK2_MAGIC;
  packet[1] = payload_length;
  packet[2] = 0U; /* incompatibility flags */
  packet[3] = 0U; /* compatibility flags */
  packet[4] = encoder->sequence++;
  packet[5] = encoder->system_id;
  packet[6] = encoder->component_id;
  packet[7] = (uint8_t)message_id;
  packet[8] = (uint8_t)(message_id >> 8U);
  packet[9] = (uint8_t)(message_id >> 16U);
  for (uint8_t i = 0U; i < payload_length; ++i)
  {
    packet[MAVLINK2_HEADER_LENGTH + i] = payload[i];
  }

  uint16_t crc = 0xFFFFU;
  for (size_t i = 1U; i < MAVLINK2_HEADER_LENGTH + payload_length; ++i)
  {
    crc_accumulate(packet[i], &crc);
  }
  crc_accumulate(crc_extra, &crc);
  packet[MAVLINK2_HEADER_LENGTH + payload_length] = (uint8_t)crc;
  packet[MAVLINK2_HEADER_LENGTH + payload_length + 1U] = (uint8_t)(crc >> 8U);
  return packet_length;
}
