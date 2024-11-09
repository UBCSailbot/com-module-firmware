/*
 *  error.h
 *
 *  Description: Provides declarations for variables and function prototypes related to error handling.
 *
 *  Created on: Mar 30, 2024
 *  Author: Moiz
 */

/* Includes ------------------------------------------------------------------*/
#include "error.h"
#include "board.h"

/* Variables ------------------------------------------------------------------*/

/* Functions ------------------------------------------------------------------*/
static void I2C_ErrorResetCycle (I2C_HandleTypeDef handle, uint16_t device_address, uint8_t register_address, uint8_t buffer)  {
    buffer[0] = register_address;
	if (HAL_I2C_Master_Transmit(&handle, (device_address << 1), buffer, 1, HAL_MAX_DELAY) != HAL_OK) {
		NVIC_SystemReset();
	} else {
        if (HAL_I2C_Master_Receive(&handle, (device_address << 1), buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
            NVIC_SystemReset();
        }
    }
    HAL_Delay(250);
}
