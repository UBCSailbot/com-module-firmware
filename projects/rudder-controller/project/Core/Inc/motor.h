/*
 * motor.h
 *
 *  Created on: Mar 6, 2024
 *      Author: Jasia
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "main.h"
#include <stddef.h>

typedef enum
{
	MR_OK = 0x00,
	MR_ERROR = 0x01

} MR_StatusTypeDef;

MR_StatusTypeDef motor_configure(void);
MR_StatusTypeDef motor_init(void);
MR_StatusTypeDef motor_set_duty_cycle(uint8_t duty_cycle);
MR_StatusTypeDef motor_move_port(void);
MR_StatusTypeDef motor_move_stbd(void); //sign of voltage reversed
MR_StatusTypeDef motor_stop(void);//set duty cycle to 0
MR_StatusTypeDef motor_turn_off(void);//shut off power completely


#endif /* INC_MOTOR_H_ */
