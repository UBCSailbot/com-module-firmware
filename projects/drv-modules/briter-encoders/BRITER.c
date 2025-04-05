/*
 * BRITER.c
 *
 *  Created on: Nov 29, 2024
 *      Author: Michael Greenough, Christian Conroy
 */

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ----------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
#include "stdbool.h"
#include "BRITER.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- PRIVATE MACROS ----------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

// UART communication and encoder polling timing configurations
#define TRANSMISSION_MAX_TIME 5000
#define DELAY_BETWEEN_TRANSMISSIONS 100
#define MAX_SCENTENCE_LENGTH 16
#define MINIMUM_SAMPLE_PERIOD 20
#define RECOVERY_GRACE_PERIOD_MS 2000
#define RECOVERY_WAIT_MS 3000
#define DATA_TIMEOUT_MS 100
#define ENCODER_NOT_READY_SENTINEL -800 //this value is returned during intermittent periods of no data
#define STARTUP_DELAY_MS 3000

//Encoder Raw data and Recovery Flag
static uint8_t inRecovery = 0;
static volatile uint16_t encoderRaw = 0;

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
void BRITER__init(BRITER* self, UART_HandleTypeDef * huartChannel, volatile uint16_t * encoderRaw, uint16_t samplePeriod) {
	//frees the input buffer
//	if (self->inputBuffer != NULL) {
//	    free(self->inputBuffer);
//	    self->inputBuffer = NULL; // Avoid double-free later
//	}

	//Copy the relevant data to the object
	self->encoderRaw = encoderRaw;
	self->huart = huartChannel;

    //ensure enoder is in fallback
    self->encoderReady = false;

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


    //Clear any flags then trigger 	a DMA reception
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_IDLE);
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_RXNE);
	__HAL_UART_CLEAR_FLAG(huartChannel, UART_FLAG_ORE);

	HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);

}

BRITER* BRITER__create(UART_HandleTypeDef * huartChannel, uint16_t samplePeriod) {
    BRITER* result = (BRITER*)malloc(sizeof(BRITER));
    if (result == NULL) {
		printf("BRITER ERROR: Failed to allocate memory for encoder object.\r\n");
		return NULL; // Prevent null pointer issues
		HAL_Delay(500); //necessary to run encoder. Puttin this in briter init kills it.
}
    memset(result, 0, sizeof(BRITER));
    // Save the sample period in the structure for later use.
    result->samplePeriod = samplePeriod;
    result->startupTime = HAL_GetTick();


    // Initialize using the internally managed encoderRaw
    BRITER__init(result, huartChannel, &encoderRaw, samplePeriod);
    return result;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- ENCODER FUNCTIONS --------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Collects our raw encoder values
uint16_t BRITER__getEncoderRaw(BRITER* self) {
	BRITER__checkErrors(self);
    return *(self->encoderRaw);
}




// Compute the 0-360 angle value
int16_t BRITER__computeAngle(BRITER *self) {
    if (self == NULL) return -1;
    uint16_t raw = BRITER__getEncoderRaw(self); // Includes error checks
    return (int16_t)((raw * 360.0f) / 1024.0f);
}



// Compute the clamped -45 to 45 passval
int16_t BRITER__clampAngle(BRITER *self) {
    if (self == NULL) return -1000; // Sentinel error value

    // Ensure angleVal is up to date
    self->angleVal = BRITER__computeAngle(self);

    int16_t signedAngle = (self->angleVal < 180) ? self->angleVal : self->angleVal - 360;

    if (signedAngle > 45) {
        return 45;
    } else if (signedAngle < -45) {
        return -45;
    } else {
        return signedAngle;
    }
}



void BRITER__reEngage(BRITER* self) {
    if (self == NULL) return;
    // Reinitialize using the values stored in the object.
    printf("BRITER: ReEngaging Encoder...\r\n");
    BRITER__init(self, self->huart, self->encoderRaw, self->samplePeriod);
}



// This function is called to power cycle the encoder. It cuts power, waits and restores power.
void BRITER__powerCycle(BRITER* self) {
    if (self == NULL) return;

    printf("BRITER: Power cycling the encoder...\r\n");

    HAL_GPIO_WritePin(ENCODER_POWER_GPIO, ENCODER_POWER_PIN, GPIO_PIN_RESET);
    HAL_Delay(RECOVERY_WAIT_MS);

    HAL_GPIO_WritePin(ENCODER_POWER_GPIO, ENCODER_POWER_PIN, GPIO_PIN_SET);
    HAL_Delay(RECOVERY_WAIT_MS);
    BRITER__reEngage(self);
}



void BRITER__checkErrors(BRITER* self) {
    if (self == NULL) return;

    uint32_t currentTime = HAL_GetTick();

    // If no valid data received shortly after boot
    if (!self->encoderReady && (currentTime - self->lastValidDataTime > RECOVERY_GRACE_PERIOD_MS)) {
        printf("BRITER ERROR: No encoder response after boot.\r\n");

        // Soft reset the UART to try again
        HAL_UART_AbortReceive(self->huart);
        HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->inputBuffer, 16);

        self->lastValidDataTime = currentTime;
        *(self->encoderRaw) = ENCODER_NOT_READY_SENTINEL;
        return;
    }
    // If encoder hasn't had time to boot, return sentinel
    if (!self->encoderReady || (currentTime - self->startupTime < STARTUP_DELAY_MS)) {
        *(self->encoderRaw) = ENCODER_NOT_READY_SENTINEL;
        return;
    }
    // Still within a recovery grace period
    if (inRecovery && (currentTime - self->lastValidDataTime < RECOVERY_GRACE_PERIOD_MS)) {
        return;
    }

    // Check for UART timeout or error
    if (!inRecovery && (currentTime - self->lastValidDataTime > DATA_TIMEOUT_MS)) {
        bool uartError = false;
        if ((__HAL_UART_GET_FLAG(self->huart, UART_FLAG_ORE) == SET) ||
            (__HAL_UART_GET_FLAG(self->huart, UART_FLAG_FE) == SET) ||
            (__HAL_UART_GET_FLAG(self->huart, UART_FLAG_NE) == SET)) {
            uartError = true;
        }

        if (uartError && (*(self->encoderRaw) == self->lastRawValue)) {
            printf("BRITER ERROR: Encoder data stagnant and UART error flag detected. Triggering recovery...\r\n");
            inRecovery = 1;
            BRITER__powerCycle(self);
        } else {
            self->lastRawValue = *(self->encoderRaw);  // Update only if new data
        }
    }

}





//handles the incoming data from the encoder. This function should be called inside HAL_UARTEx_RxEventCallback().
// It checks the message length, CRC, and data format. If everything is correct, it updates the encoder position and blinks the status LED.
void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart, uint16_t size) {
    //printf("CHECK\r\n");
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

        // If message is valid, process data
        if (bufPoint[0] == 0x01 && bufPoint[1] == 0x03 && bufPoint[2] == 0x04) {
            self->encoderRaw[0] = bufPoint[5] << 8 | bufPoint[6];

            //raises encoderready flag
            if (!self->encoderReady) {
                self->encoderReady = true;
            }
            self->lastValidDataTime = HAL_GetTick();
            BRITER__blinkStatusLED();

            // Exit recovery mode once valid data is received
            inRecovery = 0;
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

//blinks an led at a reasonable rate. Used for error catching
void BRITER__blinkStatusLED(void) {
    static uint32_t lastBlinkTime = 0;
    uint32_t now = HAL_GetTick();

    if (now - lastBlinkTime >= 500) {  // Adjust to your vibe (e.g., 1000 for 1 sec)
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
        lastBlinkTime = now;
    }
}

//zeros values of encoder
void BRITER__zeroPosition(BRITER* self){
	sendScentence(self, (uint8_t *) zeroPositionCMD, 4);
}


