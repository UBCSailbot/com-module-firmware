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
#include <stdio.h>


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

// Raw values of Encoder
static volatile uint16_t encoderRaw = 0;

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
void BRITER__init(BRITER* self, UART_HandleTypeDef * huartChannel, volatile uint16_t * encoderRaw, uint16_t samplePeriod) {

	//Copy the relevant data to the object
	self->encoderRaw = encoderRaw;
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

BRITER* BRITER__create(UART_HandleTypeDef * huartChannel, uint16_t samplePeriod) {
    BRITER* result = (BRITER*)malloc(sizeof(BRITER));
    if (result == NULL) return NULL; // Prevent null pointer issues

    // Initialize using the internally managed encoderRaw
    BRITER__init(result, huartChannel, &encoderRaw, samplePeriod);
    return result;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- ENCODER FUNCTIONS --------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Collects our raw encoder values
uint16_t BRITER__getEncoderRaw(BRITER* self) {
    return *(self->encoderRaw);
}

// Compute the 0-360 angle value
void BRITER__computeAngle(BRITER *self) {
    if (self == NULL || self->encoderRaw == NULL) return; // Safety check
    self->angleVal = (*(self->encoderRaw) * 360) / 1024;
}

// Compute the clamped -45 to 45 passval
void BRITER__computePassval(BRITER *self) {
    if (self == NULL) return; // Safety check
    int16_t signedAngle = (self->angleVal < 180) ? self->angleVal : self->angleVal - 360;
    
    if (signedAngle > 45) {
        self->passval = 45;
    } else if (signedAngle < -45) {
        self->passval = -45;
    } else {
        self->passval = signedAngle;
    }
}

//zeros values of encoder
void BRITER__zeroPosition(BRITER* self){
	sendScentence(self, (uint8_t *) zeroPositionCMD, 4);
}

void BRITER__powerCycle(BRITER* self) {
    if (self == NULL) return;

    printf("BRITER: Power cycling the encoder...\n");

    // Turn off encoder power
    HAL_GPIO_WritePin(ENCODER_POWER_GPIO, ENCODER_POWER_PIN, GPIO_PIN_RESET);
    HAL_Delay(3000);  // Wait longer to ensure full power-down

    // Turn encoder power back on
    HAL_GPIO_WritePin(ENCODER_POWER_GPIO, ENCODER_POWER_PIN, GPIO_PIN_SET);
    HAL_Delay(1000);  // Extra time for encoder to restart

    // Reset the last valid data time so it doesn't re-trigger the timeout
    self->lastValidDataTime = HAL_GetTick();

    // Reinitialize UART communication
    HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);
}


//check error
void BRITER__checkErrors(BRITER* self) {
    if (self == NULL) return;

    uint32_t currentTime = HAL_GetTick();

    // If we've already power-cycled recently, don't keep triggering
    if (currentTime - self->lastValidDataTime < 5000) {
        return;  // Allow time to stabilize before retrying
    }

    // Check for encoder timeout (100ms)
    if (currentTime - self->lastValidDataTime > 100) {
        printf("BRITER ERROR: Encoder timeout! Attempting power cycle...\n");

        // Power cycle the encoder once **Currently the power cycle function kills the entire program
       // BRITER__powerCycle(self);

        return;
    }
}



// Update BRITER__handleDMA to include angle calculations, timeout tracking, and error detection
void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart, uint16_t size) {
    if (huart->Instance == self->huart->Instance) {

        // If it is an incomplete message, ignore it
        if ((huart->RxState == HAL_UART_STATE_BUSY_RX) &&
            (__HAL_DMA_GET_COUNTER(huart->hdmarx) > 0)) {
            return;
        }

        // Check message length
        if (size != 9) {
            printf("BRITER ERROR: Received incorrect message length: %d\r\n", size);
            memset(self->inputBuffer, 0, MAX_SCENTENCE_LENGTH);
            HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);
            return;
        }

        // Check CRC
        uint8_t *bufPoint = self->inputBuffer;
        uint32_t crc = modbus_CRC(bufPoint, 7);

        if (bufPoint[7] != (crc & 0xFF) || bufPoint[8] != ((crc >> 8) & 0xFF)) {
            printf("BRITER ERROR: CRC check failed. Received: 0x%02X 0x%02X, Expected: 0x%02lX 0x%02lX\r\n",
                bufPoint[7], bufPoint[8], (unsigned long)(crc & 0xFF), (unsigned long)((crc >> 8) & 0xFF));
            memset(self->inputBuffer, 0, MAX_SCENTENCE_LENGTH);
            HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);
            return;
        }

        // **Check for errors using our new function**
        BRITER__checkErrors(self);

        // If message is valid, process data
        if (bufPoint[0] == 0x01 && bufPoint[1] == 0x03 && bufPoint[2] == 0x04) {
            self->encoderRaw[0] = bufPoint[5] << 8 | bufPoint[6];
            BRITER__computeAngle(self);
            BRITER__computePassval(self);
            self->lastValidDataTime = HAL_GetTick(); // Update last valid timestamp
        } else {
            printf("BRITER ERROR: Incorrect data format received.\r\n");
        }

        // Clear flags and request a new reception
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
        HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);
    }
}


//sends a command to the encoder
void sendScentence(BRITER* self, uint8_t *outputData, uint16_t outputLength) {
    // Allocate memory for the output buffer
    uint8_t *outputBuffer = (uint8_t *) malloc(MAX_SCENTENCE_LENGTH);
    if (outputBuffer == NULL) {
        printf("Error: Failed to allocate memory for outputBuffer\r\n");
        return;
    }

    // Format first two bytes
    outputBuffer[0] = ENCODER_ADDRESS;
    outputBuffer[1] = 0x06;

    // Copy message to the outputBuffer
    memcpy(outputBuffer + 2, outputData, outputLength);

    // Calculate and add CRC
    uint32_t checkSum = modbus_CRC(outputBuffer, outputLength + 2);
    outputBuffer[outputLength + 2] = checkSum & 0xFF;
    outputBuffer[outputLength + 3] = (checkSum >> 8) & 0xFF;

    // Transmit the message
    HAL_StatusTypeDef status = HAL_UART_Transmit(self->huart, outputBuffer, outputLength + 4, TRANSMISSION_MAX_TIME);
    if (status != HAL_OK) {
        printf("Error: UART transmission failed with status %d\r\n", status);
    }

    // Free allocated memory
    free(outputBuffer);
}





