/*
 * fsm.c
 *
 *  Created on: Mar 22, 2024
 *      Author: mlokh
 */
#include "fsm.h"
#include "controls.h"

#define FSM_PERIOD 10

static state_t fsm_state;

void run_fsm(void) {
	stateId_t next_state;

	while (1) {
		stateId_t (*func)(void) = statefunc[fsm_state.current];

		next_state = func();

		fsm_state.current = next_state;

		HAL_Delay(FSM_PERIOD);
	}
}


//-- State machine functions

stateId_t idle(void) {
	return STATE_IDLE;
}

stateId_t running(void) {
	return STATE_IDLE;
}

stateId_t error(void) {
	return STATE_IDLE;
}
