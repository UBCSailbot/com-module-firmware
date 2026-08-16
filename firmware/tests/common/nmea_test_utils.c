#include "nmea_test_utils.h"

#include <string.h>

uint8_t g_fw_enable_debug_prints = 0;
uint8_t g_fw_enable_can_prints = 0;

/**
 * @brief Convert a nibble to uppercase hex ASCII.
 *
 * @param value Value in the range 0-15.
 * @return ASCII character for the hex value.
 */
static uint8_t nmea_test_to_hex_ascii(uint8_t value) {
  if (value < 10) {
    return (uint8_t)('0' + value);
  }
  return (uint8_t)('A' + (value - 10));
}

void nmea_test_build_sentence(NMEA0183Raw *msg, char start, const char *body) {
  size_t body_len = strlen(body);
  uint8_t checksum = 0;
  size_t index = 0;

  msg->scentenceData[index++] = (uint8_t)start;
  memcpy(&msg->scentenceData[index], body, body_len);
  for (size_t i = 0; i < body_len; i++) {
    checksum ^= (uint8_t)body[i];
  }
  index += body_len;

  msg->scentenceData[index++] = '*';
  msg->scentenceData[index++] =
      nmea_test_to_hex_ascii((uint8_t)(checksum >> 4));
  msg->scentenceData[index++] =
      nmea_test_to_hex_ascii((uint8_t)(checksum & 0x0F));
  msg->scentenceData[index++] = '\r';
  msg->scentenceData[index++] = '\n';
  msg->scentenceLength = (uint8_t)index;
}

void nmea_test_set_channel_message(NMEA0183 *channel, const NMEA0183Raw *msg) {
  memset(channel, 0, sizeof(*channel));
  channel->dataBufferReadIndex = 0;
  channel->dataBufferWriteIndex = 1;
  memcpy(&channel->dataBuffer[0], msg, sizeof(*msg));
}
