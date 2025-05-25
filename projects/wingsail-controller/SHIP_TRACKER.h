/*
 * TODO: Document this and code it
 *
 *  Created on: Apr 23, 2024
 *      Author: Michael Greenough
 */

#ifndef INC_SHIP_TRACKER_H_
#define INC_SHIP_TRACKER_H_

#include <stdint.h>
#include <AIS.h>

#endif /* INC_ShipTracker_H_ */

#define TRANSMIT_BUFFER_LENGTH 127
#define SHIP_SIZE_BUFFER_LENGTH 255

typedef struct {
	uint32_t MMSINumber;
	uint16_t length;
	uint16_t width;
} SHIP_SIZE;

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

typedef struct {
	SHIP_SIZE sizeBuffer[SHIP_SIZE_BUFFER_LENGTH];
	SHIP transmitBuffer[TRANSMIT_BUFFER_LENGTH];
	uint16_t sizeBufferWriteIndex;
	uint16_t transmitBufferReadIndex;
	uint16_t transmitBufferWriteIndex;
} SHIP_TRACKER;

//create new ship tracker object
//-max number of size entries (we don't need this for positional data because we are capped by 127 CAN message so constant works fine)
SHIP_TRACKER * SHIP_TRACKER__create();

void SHIP_TRACKER__destroy(SHIP_TRACKER * self);

void SHIP_TRACKER__addShip(SHIP_TRACKER * self, AIS * ais);

uint16_t SHIP_TRACKER__numberOfShipsInTxBuffer(SHIP_TRACKER * self);

SHIP * SHIP_TRACKER__popFromTxBuffer(SHIP_TRACKER * self);

//
//void SHIP_TRACKER__addShipDimensions(SHIP_TRACKER * self, uint32_t MMSINumber, uint16_t width, uint16_t length);
////add new size data to circular buffer
////-MMSI is the key -> data is width / height
//
//uint8_t SHIP_TRACKER__addShip(SHIP_TRACKER * self, SHIP * newShipData);
////add new ship (only for message 1,2,3,18,19) to transmit buffer (this will be reset whenever we transmitted info)
////-take the long, lat, rot, as params. Create ship objects and add to buffer array
////-return true if there is room left in the buffer
//
//uint8_t SHIP_TRACKER__getShipsInBuffer(SHIP_TRACKER * self);
////get number of ships in ship buffer
//
//SHIP * SHIP_TRACKER__bufferIteratorNext(SHIP_TRACKER * self);
////get next ship in transmit buffer as SHIP object
//
//void SHIP_TRACKER__bufferIteratorReset(SHIP_TRACKER * self);
////reset iterator
//
//void SHIP_TRACKER__resetShipBuffer(SHIP_TRACKER * self);
////reset the ship transmit buffer
