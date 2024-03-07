/*
 * encoder.h
 *
 *  Created on: Mar 6, 2024
 *      Author: Jasia
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include <stddef.h>




typedef enum
{
	EN_OK =  0X00,
	EN_ERROR = 0X01,
	EN_ERROR_READ = 0x02,
	EN_ERROR_WRITE = 0x03,
	EN_CALIBREATED = 0x04,
	EN_UNCALIBRATED = 0x05
} EN_StatusTypeDef;

EN_StatusTypeDef encoder_init(void);
EN_StatusTypeDef encoder_callibrate(void);
EN_StatusTypeDef encoder_read(uint16_t* rawRead);
EN_StatusTypeDef encoder_getPosition(uint16_t* position); //calls read and does some processing

uint8_t encoder_is_reached_max(void);


#endif /* INC_ENCODER_H_ */
