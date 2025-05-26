/*
 * This module takes in an AIS object and stores the information in two circular buffers.
 * 		One is used to store all information about a ship before it is popped from the buffer.
 * 		Another is used to store the ship size information so that it can be used to associate static and dynamic data.
 * 	Both of these buffers are updated if a ship's MMSI number is already in the buffer.
 * 	This library provides a few restrictive ways to access the buffer, since there is a particular application in mind.
 *
 * 	NOTE: THIS PROGRAM IS STRICTLY SINGLE THREADED. DO NOT PUT ANY METHODS IN AN INTERRUPT.
 *
 *  Created on: Apr 23, 2024
 *      Author: Michael Greenough
 */

#ifndef INC_SHIP_TRACKER_H_
#define INC_SHIP_TRACKER_H_

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <AIS.h>

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Note:
// 1. The time complexity of adding a ship to the buffer is proportional to the sum of the following buffer sizes.
// 2. The memory utilization is also proportional to the sum of these two buffer
//The following two buffer sizes should be selected bearing these two factors in mind

//The transmit buffer length. This is used to store ship data before it is sent off.
//Currently the number of ships that can be stored in the buffer is one less than this number
#define TRANSMIT_BUFFER_LENGTH 127

//The ship size buffer length.
//This is used to store the ship dimensional information so that it can be associated to ships, since the dynamic and static data is typically transmitted separately.
//Currently the number of ship sizes that can be stored in this buffer is one less than this number.
#define SHIP_SIZE_BUFFER_LENGTH 255

//The data structure for the ships length and width
typedef struct {
	uint32_t MMSINumber;
	uint16_t length;
	uint16_t width;
} SHIP_SIZE;

//The data structure for all parameters about the ships we care about.
typedef struct {
	uint32_t MMSINumber;
	uint32_t latitude;
	uint32_t longitude;
	uint16_t speedOverGround;
	uint16_t courseOverGround;
	uint16_t heading;
	int8_t rateOfTurn;
	uint16_t length;
	uint16_t width;
} SHIP;

//The ship tracker object.
typedef struct {
	SHIP_SIZE sizeBuffer[SHIP_SIZE_BUFFER_LENGTH];
	SHIP transmitBuffer[TRANSMIT_BUFFER_LENGTH];
	uint16_t sizeBufferWriteIndex;
	uint16_t transmitBufferReadIndex;
	uint16_t transmitBufferWriteIndex;
} SHIP_TRACKER;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new SHIP_TRACKER object.
 *
 * @return an initialized SHIP_TRACKER object
 */
SHIP_TRACKER * SHIP_TRACKER__create();

/*
 * Deletes the AIS object.
 *
 * @param self Must be an initialized SHIP_TRACKER object
 */
void SHIP_TRACKER__destroy(SHIP_TRACKER * self);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------- SHIP TRACKER METHODS ----------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 *  Updates the buffers as suitable with ship information from the AIS object.
 *  This function will check the length and type to makes sure the message is complete and supported.
 *
 *  @param self Is an initialized SHIP_TRACKER object
 *  @param ais Is an AIS object with a message already passed to it.
 */
void SHIP_TRACKER__addShip(SHIP_TRACKER * self, AIS_DATA * ais);

/*
 * Gets the number of ships currently in the transmit buffer.
 *
 * @param self Is an initialized SHIP_TRACKER object
 * @return the number of ships in the transmit buffer.
 */
uint16_t SHIP_TRACKER__numberOfShipsInTxBuffer(SHIP_TRACKER * self);

/*
 * Pops a ship from the transmit buffer.
 *
 * @param self Is an initialized SHIP_TRACKER object
 * @return is a pointer to the ship that was popped from the buffer.
 */
SHIP * SHIP_TRACKER__popFromTxBuffer(SHIP_TRACKER * self);

#endif /* INC_ShipTracker_H_ */
