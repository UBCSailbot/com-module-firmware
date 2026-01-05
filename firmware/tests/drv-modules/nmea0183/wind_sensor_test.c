#include "NMEA0183.h"
#include "WIND_SENSOR.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <stdio.h>

static int g_failures = 0;

/**
 * @brief Validate MWV parsing into fixed-point fields.
 *
 * @param void
 * @return void
 */
static void test_wind_sensor_poll_parses_mwv(void) {
  NMEA0183 channel = {0};
  NMEA0183Raw msg = {0};
  nmea_test_build_sentence(&msg, '$', "IIMWV,045.0,R,10.2,N,A");
  nmea_test_set_channel_message(&channel, &msg);

  WIND_SENSOR *sensor = WIND_SENSOR__create(&channel);
  TEST_ASSERT(&g_failures, sensor != NULL);

  TEST_ASSERT(&g_failures, WIND_SENSOR__poll(sensor) == true);
  TEST_ASSERT(&g_failures, sensor->direction == (wind_direction_deg_t)450);
  TEST_ASSERT(&g_failures, sensor->reference == REFERENCE);
  TEST_ASSERT(&g_failures, sensor->speed == (wind_speed_knots_t)102);
  TEST_ASSERT(&g_failures, sensor->status == VALID);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 1);

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
