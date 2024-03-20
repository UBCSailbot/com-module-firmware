/*
 * motor.c
 *
 *  Created on: Mar 16, 2024
 *      Author: Jasia, Moiz
 */


#include "motor.h"

MR_StatusTypeDef motor_init(void){ //does configurations
	printf("motor init\n\r");
	return MR_OK;
}

MR_StatusTypeDef motor_set_duty_cycle(uint8_t duty_cycle) {
	printf("motor set duty cyle\n\r");
	return MR_OK;
}

MR_StatusTypeDef motor_move_port(void) {
	printf("motor move port\n\r");
	return MR_OK;
}

MR_StatusTypeDef motor_move_stbd(void) {
	printf("motor move stbd\n\r");
	return MR_OK;
}

MR_StatusTypeDef motor_stop(void) {
	printf("motor stop\n\r");
	return MR_OK;
}

MR_StatusTypeDef motor_turn_off(void) {
	printf("motor turn off\n\r");
	return MR_OK;
}
