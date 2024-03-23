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

		// TODO: Avoid HAL delays and run state machine using timer interrupts
		HAL_Delay(FSM_PERIOD);
	}
}


//-- State machine functions

stateId_t idle(void) {
	// TODO: Implement and get next state based on fsm diagram
	return STATE_IDLE;
}

stateId_t running(void) {
	// TODO: Implement and get next state based on fsm diagram
	return STATE_IDLE;
}

stateId_t error(void) {
	// TODO: Implement and get next state based on fsm diagram
	return STATE_IDLE;
}
