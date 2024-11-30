/*
 * This library prrovides an interface to communicate between the microcontroller and Briter Encoders.
 * It currently has the following functionality:
 *		-A method to setup the encoder and define the data rate
 *		-Call a user defined function with position data as a parameter
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
	UART_HandleTypeDef huart;
	void (*dataHandler)(uint16_t rotationalPosition);
    uint8_t inputBuffer[16];
} BRITER;

//The address used for the encoder (0x01 is default)
#define ENCODER_ADDRESS 0x01

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------- OBJECT MANAGEMENT ---------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * Creates a new BRITER object.
 *
 * @param IRQn Is an interrupt associated with the specified USART channel. The object must not be muted after calling this function.
 * @param huartChannel Is the USART channel associated with this BRITER object. The object must not be muted after calling this function.
 * @param dataHandler Is called when new position data is received and the position is sent as a parameter
 * @param DMAPiority Is the priority of the DMA channel. The priority is DMAPriority / 16. The sub priority is DMAPriority % 16.
 * @param baudRate Is the baud rate of the connected device. For by default Briter encoders are set to 9,600.
 * @return An initialized BRITER object.
 */
BRITER* BRITER__create(IRQn_Type IRQn, USART_TypeDef * huartChannel, void (*dataHandler)(uint16_t), uint8_t DMAPriority, uint32_t baudRate);

/*
 * Deletes the BRITER object.
 *
 * @param self Must be an initialized BRITER object
 */
void BRITER__destroy(BRITER* self);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------- BRITER METHODS ----------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*
 * This function should be called when there is a UART DMA receive request associated with a NMEA channel.
 * This function may call the "dataHandler" function as passed in "BRITER__create"
 *
 * @param self The BRITER object being read.
 * @param huart Is the UART data received from the DMA callback.
 */
void BRITER__handleDMA(BRITER* self, UART_HandleTypeDef *huart);