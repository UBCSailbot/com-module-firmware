#include "GPS.h"
#include "NMEA0183.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static int g_failures = 0;
static uint32_t g_tx_identifier = 0;
static uint32_t g_tx_id_type = 0;
static uint32_t g_tx_data_length = 0;
static uint8_t g_tx_payload[20];
static HAL_StatusTypeDef g_tx_status = HAL_OK;
static int g_tx_call_count = 0;

static const uint32_t GPS_FRAME_ID = 0x070U;
static const uint32_t GPS_FRAME_LENGTH = FDCAN_DLC_BYTES_20;
static const uint32_t GPS_DEGREES_SCALE = 1000000U;
static const uint32_t GPS_MINUTES_SCALE = 10000U;
static const uint32_t GPS_MINUTES_DIVISOR = 60U;
static const uint32_t GPS_LATITUDE_OFFSET = 90U;
static const uint32_t GPS_LONGITUDE_OFFSET = 180U;

/**
 * @brief Reset captured CAN transmit state.
 *
 * @param void
 * @return void
 */
static void reset_can_tx_capture(void) {
  g_tx_identifier = 0;
  g_tx_id_type = 0;
  g_tx_data_length = 0;
  memset(g_tx_payload, 0, sizeof(g_tx_payload));
  g_tx_status = HAL_OK;
  g_tx_call_count = 0;
}

static uint32_t scale_lat_lon(uint32_t degrees, uint32_t minutes_scaled,
                              uint32_t offset) {
  int64_t scaled =
      ((int64_t)degrees * GPS_DEGREES_SCALE) +
      ((int64_t)minutes_scaled * GPS_DEGREES_SCALE) /
          (GPS_MINUTES_DIVISOR * GPS_MINUTES_SCALE);
  scaled += (int64_t)offset * GPS_DEGREES_SCALE;
  return (uint32_t)scaled;
}

static uint32_t scale_lat_lon_signed(uint32_t degrees, uint32_t minutes_scaled,
                                     uint32_t offset, char hemisphere) {
  int64_t scaled =
      ((int64_t)degrees * GPS_DEGREES_SCALE) +
      ((int64_t)minutes_scaled * GPS_DEGREES_SCALE) /
          (GPS_MINUTES_DIVISOR * GPS_MINUTES_SCALE);
  if (hemisphere == 'S' || hemisphere == 'W') {
    scaled = -scaled;
  }
  scaled += (int64_t)offset * GPS_DEGREES_SCALE;
  return (uint32_t)scaled;
}

HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType,
                               uint32_t DataLength, uint8_t *DataBuffer,
                               FDCAN_HandleTypeDef *hfdcan1) {
  (void)hfdcan1;
  g_tx_identifier = Identifier;
  g_tx_id_type = IdType;
  g_tx_data_length = DataLength;
  if (DataBuffer && DataLength > 0U) {
    size_t copy_size = DataLength;
    if (copy_size > sizeof(g_tx_payload)) {
      copy_size = sizeof(g_tx_payload);
    }
    memcpy(g_tx_payload, DataBuffer, copy_size);
  }
  g_tx_call_count++;
  return g_tx_status;
}

/**
 * @brief Validate GPS__poll parses a GLL message.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_gll(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_set_channel_message(&channel, &gll_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  TEST_ASSERT(&g_failures, gps->position_valid == true);
  TEST_ASSERT(&g_failures, gps->time_valid == true);

  uint32_t expected_lat =
      scale_lat_lon(37U, 232475U, GPS_LATITUDE_OFFSET);
  uint32_t expected_lon =
      scale_lat_lon_signed(121U, 583416U, GPS_LONGITUDE_OFFSET, 'W');

  TEST_ASSERT(&g_failures, gps->latitude == expected_lat);
  TEST_ASSERT(&g_failures, gps->longitude == expected_lon);
  TEST_ASSERT(&g_failures, gps->utc_hours == 22U);
  TEST_ASSERT(&g_failures, gps->utc_minutes == 54U);
  TEST_ASSERT(&g_failures, gps->utc_seconds_ms == 44000U);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__parseMessage does not consume the channel buffer.
 *
 * @param void
 * @return void
 */
static void test_gps_parse_message_non_consuming(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);
  NMEA0183Raw *message = NULL;

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_set_channel_message(&channel, &gll_message);
  message = NMEA0183__getTopBufferItem(&channel);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, message != NULL);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 0);
  TEST_ASSERT(&g_failures, GPS__parseMessage(gps, message) == true);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 0);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll parses a VTG message.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_vtg(void) {
  NMEA0183Raw vtg_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &vtg_message, '$', "GPVTG,054.7,T,034.4,M,005.5,N,010.2,K");
  nmea_test_set_channel_message(&channel, &vtg_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  TEST_ASSERT(&g_failures, gps->speed_valid == true);
  TEST_ASSERT(&g_failures, gps->speed_kmh_thousandths == 10200U);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll parses a GGA message.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_gga(void) {
  NMEA0183Raw gga_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gga_message, '$', "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  nmea_test_set_channel_message(&channel, &gga_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  TEST_ASSERT(&g_failures, gps->position_valid == true);
  TEST_ASSERT(&g_failures, gps->time_valid == true);

  uint32_t expected_lat =
      scale_lat_lon(48U, 70380U, GPS_LATITUDE_OFFSET);
  uint32_t expected_lon =
      scale_lat_lon(11U, 310000U, GPS_LONGITUDE_OFFSET);

  TEST_ASSERT(&g_failures, gps->latitude == expected_lat);
  TEST_ASSERT(&g_failures, gps->longitude == expected_lon);
  TEST_ASSERT(&g_failures, gps->utc_hours == 12U);
  TEST_ASSERT(&g_failures, gps->utc_minutes == 35U);
  TEST_ASSERT(&g_failures, gps->utc_seconds_ms == 19000U);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll parses an RMC message.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_rmc(void) {
  NMEA0183Raw rmc_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &rmc_message, '$', "GPRMC,123520,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  nmea_test_set_channel_message(&channel, &rmc_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  TEST_ASSERT(&g_failures, gps->position_valid == true);
  TEST_ASSERT(&g_failures, gps->time_valid == true);
  TEST_ASSERT(&g_failures, gps->speed_valid == true);

  uint32_t expected_lat =
      scale_lat_lon(48U, 70380U, GPS_LATITUDE_OFFSET);
  uint32_t expected_lon =
      scale_lat_lon(11U, 310000U, GPS_LONGITUDE_OFFSET);

  TEST_ASSERT(&g_failures, gps->latitude == expected_lat);
  TEST_ASSERT(&g_failures, gps->longitude == expected_lon);
  TEST_ASSERT(&g_failures, gps->utc_hours == 12U);
  TEST_ASSERT(&g_failures, gps->utc_minutes == 35U);
  TEST_ASSERT(&g_failures, gps->utc_seconds_ms == 20000U);
  TEST_ASSERT(&g_failures, gps->speed_kmh_thousandths == 41485U);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll rejects invalid GGA fix quality.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_gga_invalid_fix(void) {
  NMEA0183Raw gga_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gga_message, '$', "GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,");
  nmea_test_set_channel_message(&channel, &gga_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == false);
  TEST_ASSERT(&g_failures, gps->position_valid == false);
  TEST_ASSERT(&g_failures, gps->time_valid == false);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll rejects invalid RMC status.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_rmc_invalid_status(void) {
  NMEA0183Raw rmc_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &rmc_message, '$', "GPRMC,123520,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
  nmea_test_set_channel_message(&channel, &rmc_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == false);
  TEST_ASSERT(&g_failures, gps->position_valid == false);
  TEST_ASSERT(&g_failures, gps->time_valid == false);
  TEST_ASSERT(&g_failures, gps->speed_valid == false);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__poll consumes one message.
 *
 * @param void
 * @return void
 */
static void test_gps_poll_consumes_message(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_set_channel_message(&channel, &gll_message);

  TEST_ASSERT(&g_failures, gps != NULL);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 0);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 1);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__CAN_transmit requires valid data.
 *
 * @param void
 * @return void
 */
static void test_gps_can_transmit_requires_valid(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183Raw vtg_message = {0};
  NMEA0183 channel = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_set_channel_message(&channel, &gll_message);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);

  reset_can_tx_capture();
  TEST_ASSERT(&g_failures, GPS__CAN_transmit(gps, &hfdcan) == HAL_ERROR);
  TEST_ASSERT(&g_failures, g_tx_call_count == 0);

  nmea_test_build_sentence(
      &vtg_message, '$', "GPVTG,054.7,T,034.4,M,005.5,N,010.2,K");
  nmea_test_set_channel_message(&channel, &vtg_message);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);

  TEST_ASSERT(&g_failures, GPS__CAN_transmit(gps, &hfdcan) == HAL_OK);
  TEST_ASSERT(&g_failures, g_tx_call_count == 1);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__parseMessage and shared channel dispatching.
 *
 * @param void
 * @return void
 */
static void test_gps_shared_channel_dispatch(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183Raw vtg_message = {0};
  NMEA0183 channel = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_build_sentence(
      &vtg_message, '$', "GPVTG,054.7,T,034.4,M,005.5,N,010.2,K");

  memset(&channel, 0, sizeof(channel));
  memcpy(&channel.dataBuffer[0], &gll_message, sizeof(gll_message));
  memcpy(&channel.dataBuffer[1], &vtg_message, sizeof(vtg_message));
  channel.dataBufferReadIndex = 0;
  channel.dataBufferWriteIndex = 2;

  TEST_ASSERT(&g_failures, gps != NULL);

  for (uint8_t index = 0; index < 2; index++) {
    NMEA0183Raw *message = NMEA0183__getTopBufferItem(&channel);
    TEST_ASSERT(&g_failures, message != NULL);
    TEST_ASSERT(&g_failures, GPS__parseMessage(gps, message) == true);
    NMEA0183__incrementReadIndex(&channel);
  }

  TEST_ASSERT(&g_failures, gps->position_valid == true);
  TEST_ASSERT(&g_failures, gps->time_valid == true);
  TEST_ASSERT(&g_failures, gps->speed_valid == true);
  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 2);

  GPS__destroy(gps);
}

/**
 * @brief Validate GPS__CAN_transmit payload packing.
 *
 * @param void
 * @return void
 */
static void test_gps_can_transmit_payload(void) {
  NMEA0183Raw gll_message = {0};
  NMEA0183Raw vtg_message = {0};
  NMEA0183 channel = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  GPS *gps = GPS__create(&channel);

  nmea_test_build_sentence(
      &gll_message, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_build_sentence(
      &vtg_message, '$', "GPVTG,054.7,T,034.4,M,005.5,N,010.2,K");

  nmea_test_set_channel_message(&channel, &gll_message);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);
  nmea_test_set_channel_message(&channel, &vtg_message);
  TEST_ASSERT(&g_failures, GPS__poll(gps) == true);

  reset_can_tx_capture();
  TEST_ASSERT(&g_failures, GPS__CAN_transmit(gps, &hfdcan) == HAL_OK);
  TEST_ASSERT(&g_failures, g_tx_call_count == 1);
  TEST_ASSERT(&g_failures, g_tx_identifier == GPS_FRAME_ID);
  TEST_ASSERT(&g_failures, g_tx_id_type == FDCAN_STANDARD_ID);
  TEST_ASSERT(&g_failures, g_tx_data_length == GPS_FRAME_LENGTH);

  uint32_t expected_lat =
      scale_lat_lon(37U, 232475U, GPS_LATITUDE_OFFSET);
  uint32_t expected_lon =
      scale_lat_lon_signed(121U, 583416U, GPS_LONGITUDE_OFFSET, 'W');

  uint32_t payload_lat = (uint32_t)g_tx_payload[0] |
                         ((uint32_t)g_tx_payload[1] << 8) |
                         ((uint32_t)g_tx_payload[2] << 16) |
                         ((uint32_t)g_tx_payload[3] << 24);
  uint32_t payload_lon = (uint32_t)g_tx_payload[4] |
                         ((uint32_t)g_tx_payload[5] << 8) |
                         ((uint32_t)g_tx_payload[6] << 16) |
                         ((uint32_t)g_tx_payload[7] << 24);
  uint32_t payload_seconds = (uint32_t)g_tx_payload[8] |
                             ((uint32_t)g_tx_payload[9] << 8) |
                             ((uint32_t)g_tx_payload[10] << 16) |
                             ((uint32_t)g_tx_payload[11] << 24);
  uint8_t payload_minutes = g_tx_payload[12];
  uint8_t payload_hours = g_tx_payload[13];
  uint32_t payload_speed = (uint32_t)g_tx_payload[16] |
                           ((uint32_t)g_tx_payload[17] << 8) |
                           ((uint32_t)g_tx_payload[18] << 16) |
                           ((uint32_t)g_tx_payload[19] << 24);

  TEST_ASSERT(&g_failures, payload_lat == expected_lat);
  TEST_ASSERT(&g_failures, payload_lon == expected_lon);
  TEST_ASSERT(&g_failures, payload_seconds == 44000U);
  TEST_ASSERT(&g_failures, payload_minutes == 54U);
  TEST_ASSERT(&g_failures, payload_hours == 22U);
  TEST_ASSERT(&g_failures, payload_speed == 10200U);

  GPS__destroy(gps);
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
  test_gps_poll_gll();
  test_gps_parse_message_non_consuming();
  test_gps_poll_vtg();
  test_gps_poll_gga();
  test_gps_poll_rmc();
  test_gps_poll_gga_invalid_fix();
  test_gps_poll_rmc_invalid_status();
  test_gps_poll_consumes_message();
  test_gps_can_transmit_requires_valid();
  test_gps_shared_channel_dispatch();
  test_gps_can_transmit_payload();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
