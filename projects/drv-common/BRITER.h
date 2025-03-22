/*
 * This library provides an interface to communicate between the microcontroller and BRITER Encoders.
 * It currently has the following functionality:
 *		-A method to setup the encoder and define the data rate
 *		-Automatically update a position register
 *		-Zero the encoder
 *
 * For this library to work as intended "BRITER__handleDMA()" must be called when "HAL_UARTEx_RxEventCallback()" is called.
 *
 *
 *  Created on: Nov 14, 2024
 *      Author: Michael Greenough
 */
 
#ifndef INC_BRITER_H_
#define INC_BRITER_H_

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- INCLUDES ---------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "stm32u5xx_hal.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- STRUCTURES ---------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

//The BRITER data type
typedef struct {
	UART_HandleTypeDef * huart;
	volatile uint16_t * encoderAngle;
    uint8_t * inputBuffer;
} BRITER;

//The address used for the encoder (0x01 is default)
#define ENCODER_ADDRESS 0x01

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new BRITER object.
 *
 * @param huartChannel Is the USART channel associated with this BRITER object. The object must not be muted after calling this function.
 * @param encoderAngle Is automatically kept up to date with the current encoder position. It may be modified at any time.
 * @param samplePeriod Is the time between the encoder position being updated in ms. 20ms is the minimum.
 * @return An initialized BRITER object.
 */
BRITER* BRITER__create(UART_HandleTypeDef * huartChannel, volatile uint16_t * encoderAngle, uint16_t samplePeriod);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------- BRITER METHODS ----------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * This function should be called when there is a UART RX callback.
 *
 * @param self The BRITER object being read.
 * @param huart Is the huart handle passed by the callback function
 * @param size Is the number of bytes received through UART
 */
void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart, uint16_t size);

/*
 * Zeroes the encoder position. This funciton takes a non-negliglbe amount of time to execute.
 * It should be apparent but this should not be called in regular sailing, only for tunning.
 * 
 * @param self Is a properly initialized BRITER object.
 */
void BRITER__zeroPosition(BRITER* self);

#endif /* INC_BRITER_H_ */
