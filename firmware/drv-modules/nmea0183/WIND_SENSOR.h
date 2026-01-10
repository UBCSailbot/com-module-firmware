/*
 * WIND_SENSOR.h
 *
 * Based on the CV7-EL wind sensor:
 * https://www.furuno.fr/docs/OPERATOR_MANUAL/LCJ-Capteurs_UserManual-CV7_ENG_18012021.pdf
 *
 *  Created on: Nov. 15, 2025
 *      Author: george-sleen
 */

#ifndef WIND_SENSOR_H_
#define WIND_SENSOR_H_

/*
 * Includes
 */

#include "NMEA0183.h"

/*
 * Type definitions
 */

typedef uint16_t tenths16_t; // Convert to fixed point by dividing by ten

typedef tenths16_t wind_direction_deg_t;
typedef tenths16_t wind_speed_knots_t;
typedef tenths16_t wind_temp_C_t;

// Based on our specific wind sensor this is what each character represents
typedef enum {
  VALID = 'A',
  INVALID = 'V',
  UNKNOWN = 0,
} wind_status_t;

// The datasheet doesn't really give enough info... This is here just to be safe
typedef enum { REFERENCE = 'R' } wind_reference_t;

/*
 * Structures
 */

// The wind sensor object
typedef struct {
  wind_direction_deg_t direction;
  wind_reference_t reference;
  wind_speed_knots_t speed;
  wind_status_t status;
  wind_temp_C_t temp;
  NMEA0183 *channel;
} WIND_SENSOR;

/*
 * Management
 */

/**
 * Creates a new WIND_SENSOR object.
 *
 * @param huartChannel Is the USART channel associated with this object. The
 * object must not be mutated after calling this function.
 * @return An initialized WIND_SENSOR object.
 */
WIND_SENSOR *WIND_SENSOR__create(NMEA0183 *nmeaChannel);

/**
 * Deletes the WIND_SENSOR object.
 *
 * @param self Must be an initialized WIND_SENSOR object
 */
void WIND_SENSOR__destroy(WIND_SENSOR *self);

/**
 * Polls the NMEA channel and updates the WIND_SENSOR object with the most
 * recent data.
 *
 * @param self must be an initialized WIND_SENSOR object (probably connected to
 * a wind sensor if you want it to do anything useful...)
 */
bool WIND_SENSOR__poll(WIND_SENSOR *self);

/*
 * Prints the current wind sensor values to stdout.
 */
void WIND_SENSOR__print(const WIND_SENSOR *self);

#endif /* WIND_SENSOR_H_ */
