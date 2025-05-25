/*
 * ShipTracker.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Michael Greenough
 */

#include <stdlib.h>
#include <string.h>
#include <SHIP_TRACKER.h>

//create new ship tracker object
//-max number of size entries (we don't need this for positional data because we are capped by 127 CAN message so constant works fine)
void SHIP_TRACKER__init(SHIP_TRACKER * self){

}


SHIP_TRACKER * SHIP_TRACKER__create(){
	SHIP_TRACKER* result = malloc(sizeof(SHIP_TRACKER));
	memset(result, 0, sizeof(SHIP_TRACKER));
	SHIP_TRACKER__init(result);
	return result;
}

void SHIP_TRACKER__reset(SHIP_TRACKER* self) {}

void SHIP_TRACKER__destroy(SHIP_TRACKER * data){
	if (data) {
	    SHIP_TRACKER__reset(data);
	    free(data);
	}
}

void updateShipInfo(SHIP * shipToUpdate, AIS * ais, SHIP_SIZE * size){
	if(size != NULL){
		shipToUpdate->width = size->width;
		shipToUpdate->length = size->length;
	}
	shipToUpdate->MMSINumber = AIS__getMMSINumber(ais);

	if(dynamicMessageType(ais) == DYNAMIC){
		shipToUpdate->longitude = AIS__getLongitude(ais);
		shipToUpdate->latitude = AIS__getLatitude(ais);
		shipToUpdate->courseOverGround = AIS_getCourseOverGround(ais);
		shipToUpdate->speedOverGround = AIS_getSpeedOverGround(ais);
		shipToUpdate->rateOfTurn = AIS_getRateOfTurn(ais);

	}
}

#define SUPPORTED 0
#define UNSUPPORTED 1

#define SIZE_MESSAGE 0
#define NOT_SIZE_MESSAGE 1

#define DYNAMIC_MESSAGE 0
#define NOT_DYNAMIC_MESSAGE 1

uint8_t supportMessageType(AIS * ais){
	if(sizeMessageType(ais) == SIZE_MESSAGE || dynamicMessageType(ais) == DYNAMIC_MESSAGE)
		return SUPPORTED;
	return UNSUPPORTED;
}

uint8_t sizeMessageType(AIS * ais){
	if(AIS__getMessageID(ais) == 5)
		return SIZE_MESSAGE;
	return NOT_SIZE_MESSAGE;
}

SHIP_SIZE * SHIP_TRACKER__addShipSize(SHIP_TRACKER * self, AIS * ais){
	//try to update a current entry
	uint32_t shipMMSINumber = AIS__getMMSINumber(ais);
	for(uint16_t sizeIterator = 0; sizeIterator < SHIP_SIZE_BUFFER_LENGTH; sizeIterator++){
		if(self->sizeBuffer[sizeIterator].MMSINumber == shipMMSINumber){
			if(sizeMessageType(ais) == SIZE_MESSAGE){
				self->sizeBuffer[sizeIterator].length = 1337;
				self->sizeBuffer[sizeIterator].width = 1337;
			}
			return &self->sizeBuffer[sizeIterator];
		}
	}

	if(sizeMessageType(ais) == SIZE_MESSAGE){

		//Add a new entry
		SHIP_SIZE * shipSize = &self->sizeBuffer[self->sizeBufferWriteIndex];
		shipSize->MMSINumber = AIS__getMMSINumber(ais);
		shipSize->width = 420;
		shipSize->length = 420;

		self->sizeBufferWriteIndex = (self->sizeBufferWriteIndex + 1) % SHIP_SIZE_BUFFER_LENGTH;
		return shipSize;
	}
	return NULL;
}

uint8_t dynamicMessageType(AIS * ais){
	if(AIS__getMessageID(ais) == 1 || AIS__getMessageID(ais) == 2 || AIS__getMessageID(ais) == 3 || AIS__getMessageID(ais) == 18){ //add 19 at some point
		return DYNAMIC_MESSAGE;
	}
	return NOT_DYNAMIC_MESSAGE;
}

void SHIP_TRACKER__addShip(SHIP_TRACKER * self, AIS * ais){
	if(supportMessageType(ais) == SUPPORTED){
		//-------------------------ship size updating--------------------------
		SHIP_SIZE * shipSize = SHIP_TRACKER__addShipSize(self, ais);


		//-----------------------update ship---------------------------------------------
		uint32_t shipMMSINumber = AIS__getMMSINumber(ais);

		uint16_t temporaryReadIndex = self->transmitBufferReadIndex;
		uint16_t writeIndex = self->transmitBufferWriteIndex;
		while(temporaryReadIndex != writeIndex){
			if(self->transmitBuffer[temporaryReadIndex].MMSINumber == shipMMSINumber){
				updateShipInfo(&self->transmitBuffer[temporaryReadIndex], ais, shipSize);
				return;
			}

			temporaryReadIndex = (temporaryReadIndex + 1) % TRANSMIT_BUFFER_LENGTH;
		}

		//----------------------------------Add new ship----------------------------------
		if(dynamicMessageType(ais) == DYNAMIC_MESSAGE){
			//get the index of the next write position
			uint8_t next = (self->transmitBufferWriteIndex + 1) % TRANSMIT_BUFFER_LENGTH;

			//check that we have room in the buffer
			if (next != self->transmitBufferReadIndex) {

				updateShipInfo(&self->transmitBuffer[self->transmitBufferWriteIndex], ais, shipSize);

				//Set the next write index
				self->transmitBufferWriteIndex = next;

				return;
			} else {
				//Buffer full dropping message
				Error_Handler();
			}
		}
	}
}


uint16_t SHIP_TRACKER__numberOfShipsInTxBuffer(SHIP_TRACKER * self){
	return (self->transmitBufferWriteIndex - self->transmitBufferReadIndex + TRANSMIT_BUFFER_LENGTH) % TRANSMIT_BUFFER_LENGTH;
}

SHIP * SHIP_TRACKER__popFromTxBuffer(SHIP_TRACKER * self){
	if (self->transmitBufferWriteIndex == self->transmitBufferReadIndex)
		Error_Handler();

	SHIP * outputShip = &self->transmitBuffer[self->transmitBufferReadIndex];

	self->transmitBufferReadIndex = (self->transmitBufferReadIndex + 1) % TRANSMIT_BUFFER_LENGTH;

	return outputShip;

}

//void SHIP_TRACKER__addShipDimensions(SHIP_TRACKER * self, uint32_t MMSINumber, uint16_t width, uint16_t length){
//	SHIP_SIZE * thisSize = NULL;
//	for(int i = 0; i < self->sizeBufferLength; i++){
//		if(self->sizeBuffer[i].MMSINumber == MMSINumber){
//			self->sizeBuffer[i] = (*thisSize);
//			break;
//		}
//		//self->sizeBuffer[i];
//	}
//	if(thisSize == NULL){
//		thisSize = &self->sizeBuffer[self->sizeBufferPosition];
//		self->sizeBufferPosition++;
//		self->sizeBufferPosition %= self->sizeBufferLength;
//	}
//	thisSize->MMSINumber = MMSINumber;
//	thisSize->length = length;
//	thisSize->width = width;
//
//	for(uint8_t i = self->iteratorPosition; i != self->transmitBufferPosition; i %= TRANSMIT_BUFFER_LENGTH){
//		if(self->transmitBuffer[i].MMSINumber == MMSINumber){
//			self->transmitBuffer[i].length = length;
//			self->transmitBuffer[i].width = width;
//
//			break;
//		}
//		i++;
//	}
//
//}
////add new size data to circular buffer
////-MMSI is the key -> data is width / height
//
//uint8_t SHIP_TRACKER__addShip(SHIP_TRACKER * self, SHIP * newShipData){
//	if(SHIP_TRACKER__getShipsInBuffer(self) >= TRANSMIT_BUFFER_LENGTH){
//		return 0;
//	}
//	//look for ship in valid part of buffer
//	//replace info except for size info if it is 0
//	for(int i = self->iteratorPosition; i != self->transmitBufferPosition; i %= TRANSMIT_BUFFER_LENGTH){
//		if(self->transmitBuffer[i].MMSINumber == newShipData->MMSINumber){
//			if(newShipData->length == 0)
//				newShipData->length = self->transmitBuffer[i].length;
//
//			if(newShipData->width == 0)
//				newShipData->width = self->transmitBuffer[i].width;
//
//			memcpy(&self->transmitBuffer[i], newShipData, sizeof(SHIP));
//
//			return 1;
//		}
//		i++;
//	}
//
//	if(newShipData->width == 0 && newShipData->length == 0)
//	{
//		for(int i = 0; i < self->sizeBufferLength; i++){
//			if(self->sizeBuffer[i].MMSINumber == newShipData->MMSINumber){
//				newShipData->width = self->sizeBuffer[i].width;
//				newShipData->length = self->sizeBuffer[i].length;
//				break;
//			}
//		}
//	}
//	memcpy(&self->transmitBuffer[self->transmitBufferPosition], newShipData, sizeof(SHIP));
//	self->transmitBufferPosition++;
//	self->transmitBufferPosition %= TRANSMIT_BUFFER_LENGTH;
//
//	if(SHIP_TRACKER__getShipsInBuffer(self) >= TRANSMIT_BUFFER_LENGTH){
//		return 0;
//	}
//	return 1;
//}
////add new ship (only for message 1,2,3,18,19) to transmit buffer (this will be reset whenever we transmitted info)
////-take the long, lat, rot, as params. Create ship objects and add to buffer array
////-return true if there is room left in the buffer
//
//uint8_t SHIP_TRACKER__getShipsInBuffer(SHIP_TRACKER * self){
//	return (TRANSMIT_BUFFER_LENGTH + 1 - (self->iteratorPosition-self->transmitBufferPosition)) % (TRANSMIT_BUFFER_LENGTH + 1);
//}
////get number of ships in ship buffer
//
//SHIP * SHIP_TRACKER__bufferIteratorNext(SHIP_TRACKER * self){
//	if(self->iteratorPosition == self->transmitBufferPosition)
//		return NULL;
//
//	SHIP * result = &self->transmitBuffer[self->iteratorPosition];
//	self->iteratorPosition++;
//	self->iteratorPosition %= TRANSMIT_BUFFER_LENGTH;
//	return result;
//}
////get next ship in transmit buffer as SHIP object if there are none null pointer
//
//void SHIP_TRACKER__bufferIteratorReset(SHIP_TRACKER * self){
//	self->iteratorPosition = 0;
//}
////reset iterator
//
//void SHIP_TRACKER__resetShipBuffer(SHIP_TRACKER * self){
//	self->transmitBufferPosition = 0;
//}
//reset the ship transmit buffer
