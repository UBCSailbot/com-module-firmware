/*
 * NMEA0183_scheduler.c
 *
 *  Created on: Jan 25, 2026
 *      Author: George Sleen
 */

#include "NMEA0183_scheduler.h"

/*
 * Constants
 */

static const uint32_t MESSAGE_GLL = 0x4C4C47;
static const uint32_t MESSAGE_VTG = 0x475456;

bool NMEA0183__scheduler_step(NMEA0183_Scheduler *scheduler,
                              uint32_t now_ms) {
  if (!scheduler || !scheduler->channel) {
    return false;
  }

  NMEA0183Raw *message = NMEA0183__getTopBufferItem(scheduler->channel);
  if (!message) {
    return false;
  }

  if (NMEA0183__checkMessage(message) != GOOD_MESSAGE) {
    NMEA0183__incrementReadIndex(scheduler->channel);
    return false;
  }

  bool parsed = false;
  uint32_t sentence_type = NMEA0183__getScentenceType(message);

  if (sentence_type == MESSAGE_VDM) {
    if (scheduler->ais_parser && scheduler->ais_batch &&
        scheduler->hfdcan1) {
      AIS_DATA *data =
          AIS__parseNMEAMessage(scheduler->ais_parser, message);
      if (data) {
        (void)AIS__CAN_process(scheduler->ais_batch, data, now_ms,
                               scheduler->hfdcan1);
        parsed = true;
      }
    }
  } else if (sentence_type == MESSAGE_MWV || sentence_type == MESSAGE_XDR) {
    if (scheduler->wind_sensor && scheduler->hfdcan1) {
      parsed = WIND_SENSOR__parseMessage(scheduler->wind_sensor, message);
      if (parsed) {
        (void)WIND_SENSOR__CAN_transmit(scheduler->wind_sensor,
                                        scheduler->hfdcan1);
      }
    }
  } else if (sentence_type == MESSAGE_GLL || sentence_type == MESSAGE_VTG) {
    if (scheduler->gps && scheduler->hfdcan1) {
      parsed = GPS__parseMessage(scheduler->gps, message);
      if (parsed) {
        (void)GPS__CAN_transmit(scheduler->gps, scheduler->hfdcan1);
      }
    }
  }

  NMEA0183__incrementReadIndex(scheduler->channel);
  return parsed;
}
