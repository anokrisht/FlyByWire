#include "mavlink_encoder.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int test_heartbeat_golden_packet(void)
{
  static const uint8_t expected[] = {
      0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x04, 0x03, 0xFB, 0xC6,
  };
  const uint8_t payload[] = {0U, 0U, 0U, 0U, 1U, 0U, 0U, 4U, 3U};
  uint8_t packet[MAVLINK2_MAX_PACKET_LENGTH] = {0};
  MavlinkEncoder encoder;
  MavlinkEncoder_Init(&encoder, 1U, 1U);
  const size_t length = MavlinkEncoder_Encode(
      &encoder, 0U, 50U, payload, sizeof(payload), packet, sizeof(packet));
  if ((length != sizeof(expected)) ||
      (memcmp(packet, expected, sizeof(expected)) != 0))
  {
    fprintf(stderr, "MAVLink HEARTBEAT golden packet mismatch\n");
    return 1;
  }
  return 0;
}

static int test_sequence_and_argument_validation(void)
{
  uint8_t packet[32] = {0};
  MavlinkEncoder encoder;
  MavlinkEncoder_Init(&encoder, 42U, 7U);
  if (MavlinkEncoder_Encode(&encoder, 0x1000000UL, 0U, NULL, 0U,
                            packet, sizeof(packet)) != 0U)
  {
    return 1;
  }
  if (MavlinkEncoder_Encode(&encoder, 1U, 0U, NULL, 0U,
                            packet, sizeof(packet)) != 12U)
  {
    return 1;
  }
  return ((packet[4] == 0U) && (packet[5] == 42U) && (packet[6] == 7U)) ? 0 : 1;
}

int main(void)
{
  if (test_heartbeat_golden_packet() != 0) return 1;
  if (test_sequence_and_argument_validation() != 0) return 1;
  puts("MAVLink encoder tests passed");
  return 0;
}
