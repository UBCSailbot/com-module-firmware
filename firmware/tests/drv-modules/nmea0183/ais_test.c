#include "AIS.h"
#include "NMEA0183.h"
#include "nmea_test_utils.h"
#include "test_assert.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;

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

  if (g_failures == 0) {
    printf("PASS\n");
    return 0;
  }
  printf("FAILURES: %d\n", g_failures);
  return 1;
}
