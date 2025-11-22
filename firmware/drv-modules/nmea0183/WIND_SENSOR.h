/*
 * WIND_SENSOR.h
 *
 *  Created on: Nov. 15, 2025
 *      Author: george-sleen
 */

#ifndef WIND_SENSOR_H_
#define WIND_SENSOR_H_

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "NMEA0183.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

// The wind sensor object
typedef struct {
	float windDirectionDegrees; // One decimal point of accuracy
	char windDirectionReference;
	float windSpeedKnots; // One decimal point of accuracy
	char status;
	float windTemperatureCelcius; // One decimal point of accuracy
	NMEA0183 nmea0183Channel;
} WIND_SENSOR;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new WIND_SENSOR object.
 *
 * @param huartChannel Is the USART channel associated with this object. The object must not be mutated after calling this function.
 * @return An initialized WIND_SENSOR object.
 */
WIND_SENSOR* WIND_SENSOR__create(NMEA0183* nmeaChannel);

/*
 * Deletes the WIND_SENSOR object.
 *
 * @param self Must be an initialized WIND_SENSOR object
 */
void WIND_SENSOR__destroy(WIND_SENSOR* self);


#endif /* WIND_SENSOR_H_ */
