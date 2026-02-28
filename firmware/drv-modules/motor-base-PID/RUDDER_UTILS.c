/*
 * RUDDER_UTILS.c
 *
 *  Created on: Nov 07, 2025
 *      Author: Michael Greenough
 */

#include "RUDDER_UTILS.h"

float currentSpeed = 0;

 void encoderZeroing(UART_HandleTypeDef * virtualSerialPort, BRITER * encoderObject) {
    char input;
    if (HAL_UART_Receive(virtualSerialPort, (uint8_t*)&input, 1, 100) != HAL_OK) {
		  return;
	  }

    if (input == 'a') {
        if (fabs(currentSpeed) > 0.0f) {
            Set_Motor_Calibrated(0.0f);
        } else {
            Set_Motor_Calibrated(CALIBRATION_SPEED);
        }
    } else if (input == 'd') {
        if (fabs(currentSpeed) > 0.0f) {
            Set_Motor_Calibrated(0.0f);
        } else {
            Set_Motor_Calibrated(-CALIBRATION_SPEED);
        }
    } else if (input == 'z') {
    	Set_Motor_Calibrated(0.0f);
    	BRITER__zeroPosition(encoderObject);
    } else {
    	Set_Motor_Calibrated(0.0f);
    }
}