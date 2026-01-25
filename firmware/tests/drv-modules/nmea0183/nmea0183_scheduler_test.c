#include "AIS.h"
#include "GPS.h"
#include "NMEA0183.h"
#include "NMEA0183_scheduler.h"
#include "WIND_SENSOR.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;
static uint32_t g_tx_identifiers[8];
static uint32_t g_tx_data_lengths[8];
static uint8_t g_tx_payloads[8][32];
static int g_tx_call_count = 0;

/**
 * @brief Reset captured CAN transmit state.
 *
 * @param void
 * @return void
 */
static void reset_can_tx_capture(void) {
  memset(g_tx_identifiers, 0, sizeof(g_tx_identifiers));
  memset(g_tx_data_lengths, 0, sizeof(g_tx_data_lengths));
  memset(g_tx_payloads, 0, sizeof(g_tx_payloads));
  g_tx_call_count = 0;
}

HAL_StatusTypeDef CAN_Transmit(uint32_t Identifier, uint32_t IdType,
                               uint32_t DataLength, uint8_t *DataBuffer,
                               FDCAN_HandleTypeDef *hfdcan1) {
  int index = g_tx_call_count;
  (void)IdType;
  (void)hfdcan1;

  if (index < (int)(sizeof(g_tx_identifiers) / sizeof(g_tx_identifiers[0]))) {
    g_tx_identifiers[index] = Identifier;
    g_tx_data_lengths[index] = DataLength;
    if (DataBuffer && DataLength > 0U) {
      size_t copy_size = DataLength;
      if (copy_size > sizeof(g_tx_payloads[index])) {
        copy_size = sizeof(g_tx_payloads[index]);
      }
      memcpy(g_tx_payloads[index], DataBuffer, copy_size);
    }
  }
  g_tx_call_count++;
  return HAL_OK;
}

/**
 * @brief Validate scheduler dispatches AIS, GPS, and wind messages.
 *
 * @param void
 * @return void
 */
static void test_scheduler_dispatches_messages(void) {
  NMEA0183Raw vdm = {0};
  NMEA0183Raw gll = {0};
  NMEA0183Raw vtg = {0};
  NMEA0183Raw mwv = {0};
  NMEA0183 channel = {0};
  FDCAN_HandleTypeDef hfdcan = {0};
  AIS_PARSER *ais_parser = AIS__create();
  AIS_CAN_BATCH ais_batch = {0};
  GPS *gps = GPS__create(&channel);
  WIND_SENSOR *wind = WIND_SENSOR__create(&channel);
  NMEA0183_Scheduler scheduler = {0};

  nmea_test_build_sentence(
      &vdm, '!',
      "AIVDM,1,1,,A,133sVfPP00PD>hRMDH@jNOvN20S8,0");
  nmea_test_build_sentence(
      &gll, '$', "GPGLL,3723.2475,N,12158.3416,W,225444,A");
  nmea_test_build_sentence(
      &vtg, '$', "GPVTG,054.7,T,034.4,M,005.5,N,010.2,K");
  nmea_test_build_sentence(&mwv, '$', "IIMWV,045.0,R,10.2,N,A");

  memset(&channel, 0, sizeof(channel));
  memcpy(&channel.dataBuffer[0], &vdm, sizeof(vdm));
  memcpy(&channel.dataBuffer[1], &gll, sizeof(gll));
  memcpy(&channel.dataBuffer[2], &vtg, sizeof(vtg));
  memcpy(&channel.dataBuffer[3], &mwv, sizeof(mwv));
  channel.dataBufferReadIndex = 0;
  channel.dataBufferWriteIndex = 4;

  scheduler.channel = &channel;
  scheduler.ais_parser = ais_parser;
  scheduler.ais_batch = &ais_batch;
  scheduler.gps = gps;
  scheduler.wind_sensor = wind;
  scheduler.hfdcan1 = &hfdcan;

  reset_can_tx_capture();
  TEST_ASSERT(&g_failures,
              NMEA0183__scheduler_step(&scheduler, 0) == true);
  TEST_ASSERT(&g_failures, ais_batch.ship_count == 1);

  TEST_ASSERT(&g_failures,
              NMEA0183__scheduler_step(&scheduler, 0) == true);
  TEST_ASSERT(&g_failures, gps->position_valid == true);

  TEST_ASSERT(&g_failures,
              NMEA0183__scheduler_step(&scheduler, 0) == true);
  TEST_ASSERT(&g_failures, gps->speed_valid == true);

  TEST_ASSERT(&g_failures,
              NMEA0183__scheduler_step(&scheduler, 0) == true);
  TEST_ASSERT(&g_failures, wind->speed == (wind_speed_knots_t)102);

  TEST_ASSERT(&g_failures, channel.dataBufferReadIndex == 4);
  TEST_ASSERT(&g_failures, g_tx_call_count >= 2);

  AIS__destroy(ais_parser);
  GPS__destroy(gps);
  WIND_SENSOR__destroy(wind);
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
  test_scheduler_dispatches_messages();

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
