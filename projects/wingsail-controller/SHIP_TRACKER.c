/*
 * ShipTracker.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Michael Greenough
 */

#include <stdlib.h>
#include <string.h>
#include <SHIP_TRACKER.h>

void SHIP_TRACKER__init(SHIP_TRACKER * self, uint16_t dimensionBufferSize);


//create new ship tracker object
//-max number of size entries (we don't need this for positional data because we are capped by 127 CAN message so constant works fine)
void SHIP_TRACKER__init(SHIP_TRACKER * self, uint16_t dimensionBufferSize){
	self->transmitBuffer = malloc(TRANSMIT_BUFFER_LENGTH * sizeof(SHIP));
	memset(self->transmitBuffer, 0, TRANSMIT_BUFFER_LENGTH * sizeof(SHIP));
	self->sizeBuffer = malloc(dimensionBufferSize * sizeof(SHIP_SIZE));
	memset(self->sizeBuffer, 0, dimensionBufferSize * sizeof(SHIP_SIZE));

	self->sizeBufferLength = dimensionBufferSize;
	self->sizeBufferPosition = 0;

	self->iteratorPosition = 0;
	self->transmitBufferPosition = 0;
}


SHIP_TRACKER * SHIP_TRACKER__create(uint16_t dimensionBufferSize){
	SHIP_TRACKER* result = malloc(sizeof(SHIP_TRACKER));
	memset(result, 0, sizeof(SHIP_TRACKER));
	SHIP_TRACKER__init(result, dimensionBufferSize);
	return result;
}

void SHIP_TRACKER__reset(SHIP_TRACKER* self) {}

void SHIP_TRACKER__destroy(SHIP_TRACKER * data){
	if (data) {
	    SHIP_TRACKER__reset(data);
	    free(data);
	}
}

void SHIP_TRACKER__addShipDimensions(SHIP_TRACKER * self, uint32_t MMSINumber, uint16_t width, uint16_t length){
	SHIP_SIZE * thisSize = NULL;
	for(int i = 0; i < self->sizeBufferLength; i++){
		if(self->sizeBuffer[i].MMSINumber == MMSINumber){
			self->sizeBuffer[i] = (*thisSize);
			break;
		}
		//self->sizeBuffer[i];
	}
	if(thisSize == NULL){
		thisSize = &self->sizeBuffer[self->sizeBufferPosition];
		self->sizeBufferPosition++;
		self->sizeBufferPosition %= self->sizeBufferLength;
	}
	thisSize->MMSINumber = MMSINumber;
	thisSize->length = length;
	thisSize->width = width;

	for(uint8_t i = self->iteratorPosition; i != self->transmitBufferPosition; i %= TRANSMIT_BUFFER_LENGTH){
		if(self->transmitBuffer[i].MMSINumber == MMSINumber){
			self->transmitBuffer[i].length = length;
			self->transmitBuffer[i].width = width;

			break;
		}
		i++;
	}

}
//add new size data to circular buffer
//-MMSI is the key -> data is width / height

uint8_t SHIP_TRACKER__addShip(SHIP_TRACKER * self, SHIP * newShipData){
	if(SHIP_TRACKER__getShipsInBuffer(self) >= TRANSMIT_BUFFER_LENGTH){
		return 0;
	}
	//look for ship in valid part of buffer
	//replace info except for size info if it is 0
	for(int i = self->iteratorPosition; i != self->transmitBufferPosition; i %= TRANSMIT_BUFFER_LENGTH){
		if(self->transmitBuffer[i].MMSINumber == newShipData->MMSINumber){
			if(newShipData->length == 0)
				newShipData->length = self->transmitBuffer[i].length;

			if(newShipData->width == 0)
				newShipData->width = self->transmitBuffer[i].width;

			memcpy(&self->transmitBuffer[i], newShipData, sizeof(SHIP));

			return 1;
		}
		i++;
	}

	if(newShipData->width == 0 && newShipData->length == 0)
	{
		for(int i = 0; i < self->sizeBufferLength; i++){
			if(self->sizeBuffer[i].MMSINumber == newShipData->MMSINumber){
				newShipData->width = self->sizeBuffer[i].width;
				newShipData->length = self->sizeBuffer[i].length;
				break;
			}
		}
	}
	memcpy(&self->transmitBuffer[self->transmitBufferPosition], newShipData, sizeof(SHIP));
	self->transmitBufferPosition++;
	self->transmitBufferPosition %= TRANSMIT_BUFFER_LENGTH;

	if(SHIP_TRACKER__getShipsInBuffer(self) >= TRANSMIT_BUFFER_LENGTH){
		return 0;
	}
	return 1;
}
//add new ship (only for message 1,2,3,18,19) to transmit buffer (this will be reset whenever we transmitted info)
//-take the long, lat, rot, as params. Create ship objects and add to buffer array
//-return true if there is room left in the buffer

uint8_t SHIP_TRACKER__getShipsInBuffer(SHIP_TRACKER * self){
	return (TRANSMIT_BUFFER_LENGTH + 1 - (self->iteratorPosition-self->transmitBufferPosition)) % (TRANSMIT_BUFFER_LENGTH + 1);
}
//get number of ships in ship buffer

SHIP * SHIP_TRACKER__bufferIteratorNext(SHIP_TRACKER * self){
	if(self->iteratorPosition == self->transmitBufferPosition)
		return NULL;

	SHIP * result = &self->transmitBuffer[self->iteratorPosition];
	self->iteratorPosition++;
	self->iteratorPosition %= TRANSMIT_BUFFER_LENGTH;
	return result;
}
//get next ship in transmit buffer as SHIP object if there are none null pointer

void SHIP_TRACKER__bufferIteratorReset(SHIP_TRACKER * self){
	self->iteratorPosition = 0;
}
//reset iterator

void SHIP_TRACKER__resetShipBuffer(SHIP_TRACKER * self){
	self->transmitBufferPosition = 0;
}
//reset the ship transmit buffer
