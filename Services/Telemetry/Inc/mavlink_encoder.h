#ifndef MAVLINK_ENCODER_H
#define MAVLINK_ENCODER_H

#include <stddef.h>
#include <stdint.h>

#define MAVLINK2_MAX_PAYLOAD_LENGTH 255U
#define MAVLINK2_MAX_PACKET_LENGTH  (10U + MAVLINK2_MAX_PAYLOAD_LENGTH + 2U)

typedef struct
{
  uint8_t system_id;
  uint8_t component_id;
  uint8_t sequence;
} MavlinkEncoder;

void MavlinkEncoder_Init(MavlinkEncoder *encoder, uint8_t system_id,
                         uint8_t component_id);

/** Encode one unsigned MAVLink 2 packet. Returns zero if an argument is invalid. */
size_t MavlinkEncoder_Encode(MavlinkEncoder *encoder, uint32_t message_id,
                             uint8_t crc_extra, const uint8_t *payload,
                             uint8_t payload_length, uint8_t *packet,
                             size_t packet_capacity);

#endif /* MAVLINK_ENCODER_H */
