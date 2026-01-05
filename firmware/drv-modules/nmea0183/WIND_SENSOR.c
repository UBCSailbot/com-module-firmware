/*
 * WIND_SENSOR.c
 *
 *  Created on: Nov. 15, 2025
 *      Author: george-sleen
 */

#include <NMEA0183.h>
#include <WIND_SENSOR.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Creates a new WIND_SENSOR object.
 *
 * @param huartChannel Is the USART channel associated with this object. The
 * object must not be mutated after calling this function.
 * @return An initialized WIND_SENSOR object.
 */
WIND_SENSOR *WIND_SENSOR__create(NMEA0183 *nmeaChannel) {
  if (!nmeaChannel) {
    return NULL;
  }

  WIND_SENSOR *self = (WIND_SENSOR *)malloc(sizeof(WIND_SENSOR));
  if (!self) {
    return NULL;
  }

  memset(self, 0, sizeof(WIND_SENSOR));

  self->channel = nmeaChannel;
  self->reference = REFERENCE;
  self->status = UNKNOWN;

  return self;
}

/**
 * Deletes the WIND_SENSOR object.
 *
 * @param self Must be an initialized WIND_SENSOR object
 */
void WIND_SENSOR__destroy(WIND_SENSOR *self) {}

/**
 * Polls the NMEA channel and updates the WIND_SENSOR object with the most
 * recent data.
 *
 * @param self must be an initialized WIND_SENSOR object (probably connected to
 * a wind sensor if you want it to do anything useful...)
 */
bool WIND_SENSOR__poll(WIND_SENSOR *self) {}
