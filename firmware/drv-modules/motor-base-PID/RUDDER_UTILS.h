/*
 *  Created on: Nov 07, 2025
 *      Author: Michael Greenough
 */

#include "BRITER.h"
#include "RUDDERPID.h"

#define CALIBRATION_SPEED 0.1

void encoderZeroing(UART_HandleTypeDef * virtualSerialPort, BRITER * encoderObject);