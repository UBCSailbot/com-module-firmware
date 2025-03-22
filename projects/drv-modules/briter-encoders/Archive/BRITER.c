/*
 * BRITER.c
 *
 *  Created on: Nov 29, 2024
 *      Author: Michael Greenough
 */

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ----------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

#include "BRITER.h"
#include <string.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PRIVATE MACROS ----------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

//Timeout duration on transmissions
#define TRANSMISSION_MAX_TIME 5000

//Delay between transmissions
#define DELAY_BETWEEN_TRANSMISSIONS 100

//Maximum length of sentence that can be received or sent
#define MAX_SCENTENCE_LENGTH 16

//Minimum period between sample request from the encoder. 20ms is the minimum recommended by the manufacturer.
#define MINIMUM_SAMPLE_PERIOD 20

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- GLOBAL VARIABLES --------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

//Set the encoder to automatically return position data
const uint8_t autoPositionCMD[] = {0x00, 0x06, 0x00, 0x01};

//Set the encoder's zero position
const uint8_t zeroPositionCMD[] = {0x00, 0x08, 0x00, 0x01};

//Set the encoder return rate (this is default 20, but will be modified in the initialization)
uint8_t encoderDataRateCMD[] = {0x00, 0x07, 0x00, 0x14};

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PFP ---------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Calculates the MODBUS CRC value.
 *
 * @param inputBuffer Is the string to calculate the CRC of.
 * @param length Is the length of the string to calculate the CRC of.
 * @return The CRC value
 */
uint32_t modbus_CRC(uint8_t * inputBuffer, uint16_t length);

/**
 * Send a message to the encoder. The message takes the following form:
 * 		Byte 0: 									Encoder Address
 * 		Byte 1: 									0x06 (Write Command)
 * 		Byte 2 to outputLength + 2: 				Your output data
 * 		Byte outputLegnth + 3 to outputLength + 4	CRC bytes
 *
 * @param self Is a properly initialized BIRTER object
 * @param outputData Is a pointer to the data to send
 * @param outputLength Is the number of outputData bytes to be sent.
 */
void sendScentence(BRITER* self, uint8_t * outputData, uint16_t outputLength);

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- HELPER FUNCTIONS ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

uint32_t modbus_CRC(uint8_t * inputBuffer, uint16_t length){
    //This entire function is ripped from the encoder manual, I don't know how it works. Don't @ me.
	uint32_t wcrc = 0xFFFF;
	for(uint16_t i = 0; i < length; i++){
		wcrc ^= (uint32_t)inputBuffer[i];
		for(uint8_t j = 0; j < 8; j++){
			if(wcrc & 0x0001){
				wcrc >>= 1;
				wcrc ^= 0xa001;
			} else {
				wcrc >>= 1;
			}
		}
	}
	return wcrc;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Initializes the BRITER object
void BRITER__init(BRITER* self, UART_HandleTypeDef * huartChannel, volatile uint16_t * encoderAngle, uint16_t samplePeriod) {

	//Copy the relevant data to the object
	self->encoderAngle = encoderAngle;
	self->huart = huartChannel;

	//Allocate and reset the inputBuffer
	self->inputBuffer = (uint8_t *) malloc(MAX_SCENTENCE_LENGTH);
	memset(self->inputBuffer, 0, MAX_SCENTENCE_LENGTH);

	//Set the last two bytes of the zero command to the samplePeriod unless less than 20ms
	if(samplePeriod >= MINIMUM_SAMPLE_PERIOD){
		encoderDataRateCMD[3] = samplePeriod;
		encoderDataRateCMD[2] = samplePeriod >> 8;
	}

	//Send the commands to enter automatic position return and set the period
    sendScentence(self, (uint8_t *) autoPositionCMD, 4);
    HAL_Delay(DELAY_BETWEEN_TRANSMISSIONS);
    sendScentence(self, encoderDataRateCMD, 4);
    HAL_Delay(DELAY_BETWEEN_TRANSMISSIONS);

    //Clear any flags then trigger a DMA reception
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_IDLE);
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_ORE);
	HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);

}

BRITER* BRITER__create(UART_HandleTypeDef * huartChannel, volatile uint16_t * encoderAngle, uint16_t samplePeriod){
    //Allocate and initialize the BRITER object
	BRITER* result = (BRITER*)malloc(sizeof(BRITER));
	BRITER__init(result, huartChannel, encoderAngle, samplePeriod);
	return result;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- ENCODER FUNCTIONS --------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart, uint16_t size){
	//Check if UART channel is that of this encoder
	if(huart->Instance == self->huart->Instance){

		//If it is an incomplete message ignore it
		if ((huart->RxState == HAL_UART_STATE_BUSY_RX) &&
			(__HAL_DMA_GET_COUNTER(huart->hdmarx) > 0)) {
			return;
		}

		//Check message is what we are looking for
		if(size == 9){
			uint8_t * bufPoint  = self->inputBuffer;
			uint32_t crc = modbus_CRC(bufPoint, 7);
			if(bufPoint[7] == (crc & 0xFF) && bufPoint[8] == ((crc >> 8) & 0xFF)){
				if(bufPoint[0] == 0x01 && bufPoint[1] == 0x03 && bufPoint[2] == 0x04){
					self->encoderAngle[0] = bufPoint[5] << 8 | bufPoint[6];
				} else {
					//wrong format
				}
			} else {
				//bad crc
			}
		} else {
			//bad length
		}

		//Clear flags and request a new reception
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
		HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);
	}
}

void BRITER__zeroPosition(BRITER* self){
	sendScentence(self, (uint8_t *) zeroPositionCMD, 4);
}

void sendScentence(BRITER* self, uint8_t * outputData, uint16_t outputLength){
	//Allocate memory for the output buffer. We do this because this function will only ever be called a handful of times
	//so dynamic memory allocation if okay.
	uint8_t* outputBuffer; 
	outputBuffer = (uint8_t*) malloc(MAX_SCENTENCE_LENGTH);
    
	//Format first two bytes
    outputBuffer[0] = ENCODER_ADDRESS;
    outputBuffer[1] = 0x06;
    
    //Copy message to the outputBuffer
    memcpy(outputBuffer + 2, outputData, outputLength);
    
    //Calculate and add CRC
    uint32_t checkSum =  modbus_CRC(outputBuffer, outputLength + 2);
    outputBuffer[outputLength + 2] = checkSum & 0xFF;
    outputBuffer[outputLength + 3] = (checkSum >> 8) & 0xFF;

    //Transmit the message
    HAL_UART_Transmit(self->huart, outputBuffer, outputLength + 4, TRANSMISSION_MAX_TIME); //In the future it would be good to check that we receive an echo from the encoder

    //Free the memory where the array is stored
    free(outputBuffer);
}

