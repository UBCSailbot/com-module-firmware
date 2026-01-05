#include "NMEA0183.h"
#include "WIND_SENSOR.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int g_failures = 0;

/**
 * @brief Capture WIND_SENSOR__print output into a buffer.
 *
 * @param sensor Wind sensor instance to print.
 * @param buffer Output buffer to fill.
 * @param buffer_size Size of the output buffer.
 * @return void
 */
static void capture_wind_sensor_print(const WIND_SENSOR *sensor, char *buffer,
                                      size_t buffer_size) {
  int saved_stdout = -1;
  FILE *tmp = NULL;

  if (!sensor || !buffer || buffer_size == 0) {
    return;
  }

  fflush(stdout);
  saved_stdout = dup(fileno(stdout));
  if (saved_stdout < 0) {
    return;
  }

  tmp = tmpfile();
  if (!tmp) {
    close(saved_stdout);
    return;
  }

  if (dup2(fileno(tmp), fileno(stdout)) < 0) {
    close(saved_stdout);
    fclose(tmp);
    return;
  }

  WIND_SENSOR__print(sensor);
  fflush(stdout);

  if (fseek(tmp, 0, SEEK_END) == 0) {
    long size = ftell(tmp);
    if (size < 0) {
      size = 0;
    }
    if ((size_t)size >= buffer_size) {
      size = (long)buffer_size - 1;
    }
    if (fseek(tmp, 0, SEEK_SET) == 0) {
      size_t read_size = fread(buffer, 1, (size_t)size, tmp);
      buffer[read_size] = '\0';
    }
  }

  dup2(saved_stdout, fileno(stdout));
  close(saved_stdout);
  fclose(tmp);
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
 * @brief Validate formatted output from WIND_SENSOR__print.
 *
 * @param void
 * @return void
 */
static void test_wind_sensor_print_format(void) {
  WIND_SENSOR sensor = {0};
  char output[128] = {0};

  sensor.direction = (wind_direction_deg_t)450;
  sensor.reference = REFERENCE;
  sensor.speed = (wind_speed_knots_t)102;
  sensor.status = VALID;
  sensor.temp = (wind_temp_C_t)0;

  capture_wind_sensor_print(&sensor, output, sizeof(output));
  TEST_ASSERT(&g_failures,
              strcmp(output, "dir=45.0 R spd=10.2 kt status=A temp=0.0 C\r\n") ==
                  0);
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
  test_wind_sensor_print_format();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
