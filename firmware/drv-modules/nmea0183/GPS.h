/*
 * GPS.h
 *
 *  Created on: Jan 25, 2026
 *      Author: codex
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "NMEA0183.h"
#include "can.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Structures
 */

typedef struct {
  uint32_t latitude;
  uint32_t longitude;
  uint32_t utc_seconds_ms;
  uint8_t utc_minutes;
  uint8_t utc_hours;
  uint32_t speed_kmh_thousandths;
  bool position_valid;
  bool time_valid;
  bool speed_valid;
  NMEA0183 *channel;
} GPS;

/*
 * Management
 */

/**
 * Creates a new GPS object.
 *
 * @param nmea_channel Is the NMEA0183 channel associated with this object.
 * @return An initialized GPS object.
 */
GPS *GPS__create(NMEA0183 *nmea_channel);

/**
 * Deletes the GPS object.
 *
 * @param self Must be an initialized GPS object.
 */
void GPS__destroy(GPS *self);

/**
 * Polls the NMEA channel and updates the GPS object with the most recent data.
 *
 * @param self Must be an initialized GPS object.
 * @return True if a supported message was parsed, false otherwise.
 */
bool GPS__poll(GPS *self);

/**
 * Parse a single NMEA0183 message without consuming the channel buffer.
 *
 * @param self Must be an initialized GPS object.
 * @param message Must be a valid NMEA0183 message.
 * @return True if a supported message was parsed, false otherwise.
 */
bool GPS__parseMessage(GPS *self, NMEA0183Raw *message);

/**
 * Transmit the GPS data over CAN (0x070).
 *
 * @param self An initialized GPS object.
 * @param hfdcan1 A CAN handle.
 * @return HAL_OK on success or a HAL error code.
 */
HAL_StatusTypeDef GPS__CAN_transmit(const GPS *self,
                                    FDCAN_HandleTypeDef *hfdcan1);

#endif /* INC_GPS_H_ */
