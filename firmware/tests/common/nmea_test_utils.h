#ifndef NMEA_TEST_UTILS_H
#define NMEA_TEST_UTILS_H

#include "NMEA0183.h"

/**
 * @brief Build a valid NMEA0183 sentence with checksum and CRLF.
 *
 * @param msg Output sentence container.
 * @param start Start character ('$' or '!').
 * @param body Sentence body without checksum or terminators.
 * @return void
 */
void nmea_test_build_sentence(NMEA0183Raw *msg, char start, const char *body);

/**
 * @brief Create a channel buffer with a single message.
 *
 * @param channel Channel buffer to update.
 * @param msg Message to store.
 * @return void
 */
void nmea_test_set_channel_message(NMEA0183 *channel, const NMEA0183Raw *msg);

#endif
