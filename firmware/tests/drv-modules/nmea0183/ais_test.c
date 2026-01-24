#include "AIS.h"
#include "NMEA0183.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static int g_failures = 0;
static uint32_t g_tx_identifiers[4];
static uint32_t g_tx_id_types[4];
static uint32_t g_tx_data_lengths[4];
static uint8_t g_tx_payloads[4][32];
static uint8_t g_tx_payload_sizes[4];
static HAL_StatusTypeDef g_tx_statuses[4];
static int g_tx_status_count = 0;
static int g_tx_call_count = 0;

static const uint32_t AIS_FRAME_ID = 0x060U;
static const uint32_t AIS_FRAME_LENGTH = FDCAN_DLC_BYTES_32;
static const uint32_t AIS_LATITUDE_OFFSET = 90U;
static const uint32_t AIS_LONGITUDE_OFFSET = 180U;
static const uint32_t AIS_DEGREES_SCALE = 1000000U;
static const uint32_t AIS_AIS_SCALE = 600000U;
static const uint16_t AIS_SOG_UNAVAILABLE = 1023U;
static const uint16_t AIS_COG_UNAVAILABLE = 3600U;
static const uint16_t AIS_HEADING_UNAVAILABLE = 511U;
static const int8_t AIS_ROT_UNAVAILABLE = -128;
static const uint32_t AIS_32BIT_MAX = 0xFFFFFFFFU;

/**
 * @brief Reset captured CAN transmit state.
 *
 * @param void
 * @return void
 */
static void reset_can_tx_capture(void) {
  memset(g_tx_identifiers, 0, sizeof(g_tx_identifiers));
  memset(g_tx_id_types, 0, sizeof(g_tx_id_types));
  memset(g_tx_data_lengths, 0, sizeof(g_tx_data_lengths));
  memset(g_tx_payloads, 0, sizeof(g_tx_payloads));
  memset(g_tx_payload_sizes, 0, sizeof(g_tx_payload_sizes));
  memset(g_tx_statuses, 0, sizeof(g_tx_statuses));
  g_tx_status_count = 0;
  g_tx_call_count = 0;
}

static uint32_t convert_latitude(int32_t ais_lat) {
  if (ais_lat == INT32_MAX) {
    return AIS_32BIT_MAX;
  }
  int64_t scaled = ((int64_t)ais_lat * AIS_DEGREES_SCALE) / AIS_AIS_SCALE;
  scaled += (int64_t)AIS_LATITUDE_OFFSET * AIS_DEGREES_SCALE;
  if (scaled < 0) {
    return 0;
  }
  if (scaled > (int64_t)AIS_32BIT_MAX) {
    return AIS_32BIT_MAX;
  }
  return (uint32_t)scaled;
}

static uint32_t convert_longitude(int32_t ais_lon) {
  if (ais_lon == INT32_MAX) {
    return AIS_32BIT_MAX;
  }
  int64_t scaled = ((int64_t)ais_lon * AIS_DEGREES_SCALE) / AIS_AIS_SCALE;
  scaled += (int64_t)AIS_LONGITUDE_OFFSET * AIS_DEGREES_SCALE;
  if (scaled < 0) {
    return 0;
  }
  if (scaled > (int64_t)AIS_32BIT_MAX) {
    return AIS_32BIT_MAX;
  }
  return (uint32_t)scaled;
}

/**
 * @brief Build expected CAN payload for AIS__CAN_transmit.
 *
 * @param data AIS data instance to read fields from.
 * @param ship_idx Index of the ship.
 * @param total_ships Total ships in the cycle.
 * @param expected Output payload buffer (32 bytes).
 * @return void
 */
static void build_expected_payload(const AIS_DATA *data, uint8_t ship_idx,
                                   uint8_t total_ships,
                                   uint8_t expected[32]) {
  uint32_t mmsi = AIS__getMMSINumber((AIS_DATA *)data);
  uint32_t latitude = convert_latitude(AIS__getLatitude((AIS_DATA *)data));
  uint32_t longitude = convert_longitude(AIS__getLongitude((AIS_DATA *)data));

  uint16_t speed_over_ground = AIS__getSpeedOverGround((AIS_DATA *)data);
  if (speed_over_ground == UINT16_MAX) {
    speed_over_ground = AIS_SOG_UNAVAILABLE;
  }
  uint16_t course_over_ground = AIS__getCourseOverGround((AIS_DATA *)data);
  if (course_over_ground == UINT16_MAX) {
    course_over_ground = AIS_COG_UNAVAILABLE;
  }
  uint16_t heading = AIS__getTrueHeading((AIS_DATA *)data);
  if (heading == UINT16_MAX) {
    heading = AIS_HEADING_UNAVAILABLE;
  }
  int8_t rate_of_turn = AIS__getRateOfTurn((AIS_DATA *)data);
  if (rate_of_turn == INT8_MIN) {
    rate_of_turn = AIS_ROT_UNAVAILABLE;
  }

  uint16_t dimension_a = AIS__getDimensionA((AIS_DATA *)data);
  uint16_t dimension_b = AIS__getDimensionB((AIS_DATA *)data);
  uint8_t dimension_c = AIS__getDimensionC((AIS_DATA *)data);
  uint8_t dimension_d = AIS__getDimensionD((AIS_DATA *)data);

  uint16_t length = 0;
  uint16_t width = 0;
  if (dimension_a != UINT16_MAX && dimension_b != UINT16_MAX) {
    length = (uint16_t)(dimension_a + dimension_b);
  }
  if (dimension_c != UINT8_MAX && dimension_d != UINT8_MAX) {
    width = (uint16_t)(dimension_c + dimension_d);
  }

  memset(expected, 0, 32);

  expected[0] = (uint8_t)(mmsi & 0xFF);
  expected[1] = (uint8_t)((mmsi >> 8) & 0xFF);
  expected[2] = (uint8_t)((mmsi >> 16) & 0xFF);
  expected[3] = (uint8_t)((mmsi >> 24) & 0xFF);

  expected[4] = (uint8_t)(latitude & 0xFF);
  expected[5] = (uint8_t)((latitude >> 8) & 0xFF);
  expected[6] = (uint8_t)((latitude >> 16) & 0xFF);
  expected[7] = (uint8_t)((latitude >> 24) & 0xFF);

  expected[8] = (uint8_t)(longitude & 0xFF);
  expected[9] = (uint8_t)((longitude >> 8) & 0xFF);
  expected[10] = (uint8_t)((longitude >> 16) & 0xFF);
  expected[11] = (uint8_t)((longitude >> 24) & 0xFF);

  expected[12] = (uint8_t)(speed_over_ground & 0xFF);
  expected[13] = (uint8_t)((speed_over_ground >> 8) & 0xFF);

  expected[14] = (uint8_t)(course_over_ground & 0xFF);
  expected[15] = (uint8_t)((course_over_ground >> 8) & 0xFF);

  expected[16] = (uint8_t)(heading & 0xFF);
  expected[17] = (uint8_t)((heading >> 8) & 0xFF);

  expected[18] = (uint8_t)rate_of_turn;

  expected[19] = (uint8_t)(length & 0xFF);
  expected[20] = (uint8_t)((length >> 8) & 0xFF);

  expected[21] = (uint8_t)(width & 0xFF);
  expected[22] = (uint8_t)((width >> 8) & 0xFF);

  expected[23] = ship_idx;
  expected[24] = total_ships;
}

HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType,
                               uint32_t DataLength, uint8_t *DataBuffer,
                               FDCAN_HandleTypeDef *hfdcan1) {
  int index = g_tx_call_count;

  (void)hfdcan1;

  if (index < (int)(sizeof(g_tx_identifiers) / sizeof(g_tx_identifiers[0]))) {
    g_tx_identifiers[index] = Identifier;
    g_tx_id_types[index] = IdType;
    g_tx_data_lengths[index] = DataLength;
    g_tx_payload_sizes[index] = (uint8_t)DataLength;
    if (DataBuffer && DataLength > 0U) {
      size_t copy_size = (size_t)DataLength;
      if (copy_size > sizeof(g_tx_payloads[index])) {
        copy_size = sizeof(g_tx_payloads[index]);
      }
      memcpy(g_tx_payloads[index], DataBuffer, copy_size);
    }
  }

  g_tx_call_count++;

  if (index < g_tx_status_count) {
    return g_tx_statuses[index];
  }
  return HAL_OK;
}

/**
 * @brief Validate parsing of a single-part AIS message.
 *
 * @param void
 * @return void
 */
static void test_ais_single_sentence_parse(void) {
  AIS_PARSER *parser = AIS__create();
  NMEA0183Raw msg = {0};

  nmea_test_build_sentence(
      &msg, '!',
      "AIVDM,1,1,,A,133sVfPP00PD>hRMDH@jNOvN20S8,0");
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);

  TEST_ASSERT(&g_failures, parser != NULL);
  AIS_DATA *data = AIS__parseNMEAMessage(parser, &msg);
  TEST_ASSERT(&g_failures, data != NULL);
  TEST_ASSERT(&g_failures, data->dataLength == strlen((char *)data->sixBitData));
  TEST_ASSERT(&g_failures, AIS__getMessageID(data) == 1);

  AIS__destroy(parser);
}

/**
 * @brief Validate multi-sentence AIS assembly and length checks.
 *
 * @param void
 * @return void
 */
static void test_ais_multi_sentence_parse(void) {
  AIS_PARSER *parser = AIS__create();
  NMEA0183Raw part1 = {0};
  NMEA0183Raw part2 = {0};

  nmea_test_build_sentence(
      &part1, '!',
      "AIVDM,2,1,9,B,53nFBv01SJ<thHp6220H4heHTf2222222222221?50:454o<`9QSlUDp,0");
  nmea_test_build_sentence(&part2, '!',
                           "AIVDM,2,2,9,B,888888888888880,2");

  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&part1) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&part2) == GOOD_MESSAGE);

  TEST_ASSERT(&g_failures, parser != NULL);
  TEST_ASSERT(&g_failures, AIS__parseNMEAMessage(parser, &part1) == NULL);

  AIS_DATA *data = AIS__parseNMEAMessage(parser, &part2);
  TEST_ASSERT(&g_failures, data != NULL);
  TEST_ASSERT(&g_failures, data->dataLength == 71);
  TEST_ASSERT(&g_failures, AIS__getMessageID(data) == 5);
  TEST_ASSERT(&g_failures, AIS__checkLength(data) == true);

  AIS__destroy(parser);
}

/**
 * @brief Validate AIS__CAN_transmit rejects null inputs.
 *
 * @param void
 * @return void
 */
static void test_ais_can_transmit_rejects_null(void) {
  AIS_DATA data = {0};
  FDCAN_HandleTypeDef hfdcan = {0};

  reset_can_tx_capture();
  TEST_ASSERT(&g_failures,
              AIS__CAN_transmit(NULL, 0, 0, &hfdcan) == HAL_ERROR);
  TEST_ASSERT(&g_failures, g_tx_call_count == 0);

  TEST_ASSERT(&g_failures,
              AIS__CAN_transmit(&data, 0, 0, NULL) == HAL_ERROR);
  TEST_ASSERT(&g_failures, g_tx_call_count == 0);
}

/**
 * @brief Validate AIS__CAN_transmit packs dynamic message payload fields.
 *
 * @param void
 * @return void
 */
static void test_ais_can_transmit_dynamic_payload(void) {
  AIS_PARSER *parser = AIS__create();
  NMEA0183Raw msg = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  uint8_t expected_payload[32] = {0};

  nmea_test_build_sentence(
      &msg, '!',
      "AIVDM,1,1,,A,133sVfPP00PD>hRMDH@jNOvN20S8,0");
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&msg) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, parser != NULL);

  AIS_DATA *data = AIS__parseNMEAMessage(parser, &msg);
  TEST_ASSERT(&g_failures, data != NULL);

  reset_can_tx_capture();
  build_expected_payload(data, 2, 5, expected_payload);

  TEST_ASSERT(&g_failures,
              AIS__CAN_transmit(data, 2, 5, &hfdcan) == HAL_OK);
  TEST_ASSERT(&g_failures, g_tx_call_count == 1);
  TEST_ASSERT(&g_failures, g_tx_identifiers[0] == AIS_FRAME_ID);
  TEST_ASSERT(&g_failures, g_tx_id_types[0] == FDCAN_STANDARD_ID);
  TEST_ASSERT(&g_failures, g_tx_data_lengths[0] == AIS_FRAME_LENGTH);
  TEST_ASSERT(&g_failures, g_tx_payload_sizes[0] == 32);
  TEST_ASSERT(&g_failures,
              memcmp(g_tx_payloads[0], expected_payload, 32) == 0);

  AIS__destroy(parser);
}

/**
 * @brief Validate AIS__CAN_transmit packs defaults for static messages.
 *
 * @param void
 * @return void
 */
static void test_ais_can_transmit_static_defaults(void) {
  AIS_PARSER *parser = AIS__create();
  NMEA0183Raw part1 = {0};
  NMEA0183Raw part2 = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  uint8_t expected_payload[32] = {0};

  nmea_test_build_sentence(
      &part1, '!',
      "AIVDM,2,1,9,B,53nFBv01SJ<thHp6220H4heHTf2222222222221?50:454o<`9QSlUDp,0");
  nmea_test_build_sentence(&part2, '!',
                           "AIVDM,2,2,9,B,888888888888880,2");

  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&part1) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, NMEA0183__checkMessage(&part2) == GOOD_MESSAGE);
  TEST_ASSERT(&g_failures, parser != NULL);

  TEST_ASSERT(&g_failures, AIS__parseNMEAMessage(parser, &part1) == NULL);
  AIS_DATA *data = AIS__parseNMEAMessage(parser, &part2);
  TEST_ASSERT(&g_failures, data != NULL);

  reset_can_tx_capture();
  build_expected_payload(data, 1, 3, expected_payload);

  TEST_ASSERT(&g_failures,
              AIS__CAN_transmit(data, 1, 3, &hfdcan) == HAL_OK);
  TEST_ASSERT(&g_failures, g_tx_call_count == 1);
  TEST_ASSERT(&g_failures, g_tx_identifiers[0] == AIS_FRAME_ID);
  TEST_ASSERT(&g_failures, g_tx_id_types[0] == FDCAN_STANDARD_ID);
  TEST_ASSERT(&g_failures, g_tx_data_lengths[0] == AIS_FRAME_LENGTH);
  TEST_ASSERT(&g_failures, g_tx_payload_sizes[0] == 32);
  TEST_ASSERT(&g_failures,
              memcmp(g_tx_payloads[0], expected_payload, 32) == 0);

  AIS__destroy(parser);
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
  test_ais_single_sentence_parse();
  test_ais_multi_sentence_parse();
  test_ais_can_transmit_rejects_null();
  test_ais_can_transmit_dynamic_payload();
  test_ais_can_transmit_static_defaults();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
