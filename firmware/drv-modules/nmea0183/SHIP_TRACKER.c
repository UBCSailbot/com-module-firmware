/*
 * ShipTracker.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Michael Greenough
 */

#include <stdlib.h>
#include <string.h>
#include <SHIP_TRACKER.h>

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PFP ---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Updates the ship size buffer with new ship size data if the AIS object has static data,
 * and returns the ship size object associated to the MMSI number of the AIS object.
 *
 * @param self Is an initialized SHIP_TRACKER object.
 * @param ais Is an AIS object with a supported message type.
 * @return A pointer to the SHIP_SIZE object with MMSI number matching that of the AIS object. NULL pointer otherwise.
 */
SHIP_SIZE * SHIP_TRACKER__addShipSize(SHIP_TRACKER * self, AIS_DATA * ais);

/*
 * Updates the TX buffer with new information
 *
 * @param shipToUpdate Is a pointer to a SHIP in the active area of the TX buffer to be updated
 * @param ais Is an initialized AIS object with a supported message type.
 * @param size Is size information to update this ship with. If this is a NULL pointer no size information will be updated.
 */
void updateTxBuffer(SHIP * shipToUpdate, AIS_DATA * ais, SHIP_SIZE * size);

void Error_Handler(void);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//create new ship tracker object
void SHIP_TRACKER__init(SHIP_TRACKER * self){
	memset(self, 0, sizeof(SHIP_TRACKER));
}

SHIP_TRACKER * SHIP_TRACKER__create(){
	SHIP_TRACKER* result = malloc(sizeof(SHIP_TRACKER));
	SHIP_TRACKER__init(result);
	return result;
}

void SHIP_TRACKER__reset(SHIP_TRACKER* self) {
	memset(self, 0, sizeof(SHIP_TRACKER));
}

void SHIP_TRACKER__destroy(SHIP_TRACKER * data){
	if (data) {
	    SHIP_TRACKER__reset(data);
	    free(data);
	}
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- HELPER FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

SHIP_SIZE * SHIP_TRACKER__addShipSize(SHIP_TRACKER * self, AIS_DATA * ais){
	//First see if there is a matching entry in the ship size buffer and update it if we have static data, or just return it.
	uint32_t shipMMSINumber = AIS__getMMSINumber(ais);
	for(uint16_t sizeIterator = 0; sizeIterator < SHIP_SIZE_BUFFER_LENGTH; sizeIterator++){
		if(self->sizeBuffer[sizeIterator].MMSINumber == shipMMSINumber){
			if(AIS__isSizeMessage(ais) == true){
				self->sizeBuffer[sizeIterator].length = AIS__getDimensionA(ais) + AIS__getDimensionB(ais); //TODO: edit longitude and latitude????
				self->sizeBuffer[sizeIterator].width = AIS__getDimensionC(ais) + AIS__getDimensionD(ais);
			}
			return &self->sizeBuffer[sizeIterator];
		}
	}

	//Secondly if we have static data add it to the size buffer and return it
	if(AIS__isSizeMessage(ais) == true){
		SHIP_SIZE * shipSize = &self->sizeBuffer[self->sizeBufferWriteIndex];

		shipSize->MMSINumber = AIS__getMMSINumber(ais);
		shipSize->width = AIS__getDimensionA(ais) + AIS__getDimensionB(ais);
		shipSize->length = AIS__getDimensionC(ais) + AIS__getDimensionD(ais);

		self->sizeBufferWriteIndex = (self->sizeBufferWriteIndex + 1) % SHIP_SIZE_BUFFER_LENGTH;
		return shipSize;
	}

	//Lastly return a NULL pointer if there is no size data in buffer and it is not a static message type.
	return NULL;
}

void updateTxBuffer(SHIP * shipToUpdate, AIS_DATA * ais, SHIP_SIZE * size){
	//Update the MMSI number
	shipToUpdate->MMSINumber = AIS__getMMSINumber(ais);

	//Update the size if it is not NULL
	if(size != NULL){
		shipToUpdate->width = size->width;
		shipToUpdate->length = size->length;
	}

	//Update the dynamic data if we have a dynamic message type.
	if(AIS__isDynamicMessage(ais) == true){
		shipToUpdate->longitude = AIS__getLongitude(ais);
		shipToUpdate->latitude = AIS__getLatitude(ais);
		shipToUpdate->courseOverGround = AIS__getCourseOverGround(ais);
		shipToUpdate->speedOverGround = AIS__getSpeedOverGround(ais);
		shipToUpdate->rateOfTurn = AIS__getRateOfTurn(ais);
		shipToUpdate->heading = AIS__getTrueHeading(ais);
	}
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- DATA PARSING FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void SHIP_TRACKER__addShip(SHIP_TRACKER * self, AIS_DATA * ais){
	if(AIS__isSupportedMessage(ais) == true && AIS__checkLength(ais) == true){
		//Update the ship size / get the ship size
		SHIP_SIZE * shipSize = SHIP_TRACKER__addShipSize(self, ais);


		//Try to update an existing entry
		uint32_t shipMMSINumber = AIS__getMMSINumber(ais);

		uint16_t temporaryReadIndex = self->transmitBufferReadIndex;
		uint16_t writeIndex = self->transmitBufferWriteIndex;
		while(temporaryReadIndex != writeIndex){
			if(self->transmitBuffer[temporaryReadIndex].MMSINumber == shipMMSINumber){
				updateTxBuffer(&self->transmitBuffer[temporaryReadIndex], ais, shipSize);
				return;
			}

			temporaryReadIndex = (temporaryReadIndex + 1) % TRANSMIT_BUFFER_LENGTH;
		}

		//Add a new entry
		if(AIS__isDynamicMessage(ais) == true){
			uint8_t next = (self->transmitBufferWriteIndex + 1) % TRANSMIT_BUFFER_LENGTH;

			if (next != self->transmitBufferReadIndex) {

				memset(&self->transmitBuffer[self->transmitBufferWriteIndex], 0, sizeof(SHIP));

				updateTxBuffer(&self->transmitBuffer[self->transmitBufferWriteIndex], ais, shipSize);

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
