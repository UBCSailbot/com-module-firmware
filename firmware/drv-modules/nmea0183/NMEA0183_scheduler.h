/*
 * NMEA0183_scheduler.h
 *
 *  Created on: Jan 25, 2026
 *      Author: George Sleen
 */

#ifndef INC_NMEA0183_SCHEDULER_H_
#define INC_NMEA0183_SCHEDULER_H_

#include "AIS.h"
#include "GPS.h"
#include "NMEA0183.h"
#include "WIND_SENSOR.h"
#include "can.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Structures
 */

typedef struct {
  NMEA0183 *channel;
  AIS_PARSER *ais_parser;
  AIS_CAN_BATCH *ais_batch;
  GPS *gps;
  WIND_SENSOR *wind_sensor;
  FDCAN_HandleTypeDef *hfdcan1;
} NMEA0183_Scheduler;

/*
 * Scheduler
 */

/**
 * Process one NMEA0183 message from a shared channel.
 * This function consumes at most one message from the channel buffer.
 *
 * @param scheduler Scheduler configuration.
 * @param now_ms Current time in milliseconds.
 * @return True if a supported message was parsed, false otherwise.
 */
bool NMEA0183__scheduler_step(NMEA0183_Scheduler *scheduler,
                              uint32_t now_ms);

#endif /* INC_NMEA0183_SCHEDULER_H_ */
