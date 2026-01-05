#include "NMEA0183.h"
#include "WIND_SENSOR.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

/**
 * @brief Record a test assertion failure.
 *
 * @param condition Expression result to evaluate.
 * @param expr String form of the expression.
 * @param file Source file where the assertion occurred.
 * @param line Line number where the assertion occurred.
 * @return void
 */
static void test_assert(int condition, const char *expr, const char *file,
                        int line) {
  if (!condition) {
    printf("FAIL: %s:%d: %s\n", file, line, expr);
    g_failures++;
  }
}

#define TEST_ASSERT(cond) test_assert((cond), #cond, __FILE__, __LINE__)

/**
 * @brief Convert a nibble to uppercase hex ASCII.
 *
 * @param value Value in the range 0-15.
 * @return ASCII character for the hex value.
 */
static uint8_t to_hex_ascii(uint8_t value) {
  if (value < 10) {
    return (uint8_t)('0' + value);
  }
  return (uint8_t)('A' + (value - 10));
}

/**
 * @brief Build a valid NMEA0183 sentence with checksum and CRLF.
 *
 * @param msg Output sentence container.
 * @param start Start character ('$' or '!').
 * @param body Sentence body without checksum or terminators.
 * @return void
 */
static void build_sentence(NMEA0183Raw *msg, char start, const char *body) {
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
  msg->scentenceData[index++] = to_hex_ascii((uint8_t)(checksum >> 4));
  msg->scentenceData[index++] = to_hex_ascii((uint8_t)(checksum & 0x0F));
  msg->scentenceData[index++] = '\r';
  msg->scentenceData[index++] = '\n';
  msg->scentenceLength = (uint8_t)index;
}

/**
 * @brief Create a channel buffer with a single message.
 *
 * @param channel Channel buffer to update.
 * @param msg Message to store.
 * @return void
 */
static void set_channel_message(NMEA0183 *channel, const NMEA0183Raw *msg) {
  memset(channel, 0, sizeof(*channel));
  channel->dataBufferReadIndex = 0;
  channel->dataBufferWriteIndex = 1;
  memcpy(&channel->dataBuffer[0], msg, sizeof(*msg));
}

/**
 * @brief Validate MWV parsing into fixed-point fields.
 *
 * @param void
 * @return void
 */
static void test_wind_sensor_poll_parses_mwv(void) {
  NMEA0183 channel = {0};
  NMEA0183Raw msg = {0};
  build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");
  set_channel_message(&channel, &msg);

  WIND_SENSOR *sensor = WIND_SENSOR__create(&channel);
  TEST_ASSERT(sensor != NULL);

  TEST_ASSERT(WIND_SENSOR__poll(sensor) == true);
  TEST_ASSERT(sensor->direction == (wind_direction_deg_t)450);
  TEST_ASSERT(sensor->reference == REFERENCE);
  TEST_ASSERT(sensor->speed == (wind_speed_knots_t)102);
  TEST_ASSERT(sensor->status == VALID);
  TEST_ASSERT(channel.dataBufferReadIndex == 1);

  WIND_SENSOR__destroy(sensor);
}

/**
 * @brief Stub Error_Handler for host tests.
 *
 * @param void
 * @return void
 */
void Error_Handler(void) {
  g_failures++;
}

/**
 * @brief Test runner entry point.
 *
 * @param void
 * @return 0 when all tests pass, nonzero otherwise.
 */
int main(void) {
  test_wind_sensor_poll_parses_mwv();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
