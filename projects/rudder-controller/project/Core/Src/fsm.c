/*
 * fsm.c
 *
 *  Created on: Mar 22, 2024
 *      Author: mlokh, jazreen
 */
#include "fsm.h"
#include "controls.h"

#define FSM_PERIOD 10
#define SMALL_ANGLE 0.2
#define MAX_WAIT 0.5

static state_t fsm_state; //initializes everything to 0

void run_fsm(void) {
	stateId_t next_state;

	while (1) {
		//we are declaring function pointer named func that takes no arguments and returns stateId_t
		// initialized with the function pointer from the statefunc array corresponding to the current state
		stateId_t (*func)(void) = statefunc[fsm_state.current];

		//calls the function pointed to by func
		//stores the return value/result in next state variable
		next_state = func();

		//updates the current state with the returned state id
		fsm_state.current = next_state;

		// TODO: Avoid HAL delays and run state machine using timer interrupts
		HAL_Delay(FSM_PERIOD);
	}
}


//-- State machine functions

stateId_t idle(void) {
	// TODO: Implement and get next state based on fsm diagram

	uint16_t bearing;
	double desired_heading, error;
	stateId_t next_state;
	EC_StatusTypeDef ec_status;

	desired_heading = get_desired_heading(); //from base library//do error check

	ec_status = ecompass_getBearing(&bearing);
	if (ec_status != EC_OK) return ERROR;

	error = desired_heading - bearing;

	if (heading_received() && (error >= SMALL_ANGLE)){ //make heading_received() a function in controls.c/h
		motor_turn_on();
		next_state = STATE_RUNNING;
	}

	else { //make heading_received() a function in controls.c/h
			motor_turn_on();
			next_state = STATE_IDLE;
	}

	return next_state;
}

stateId_t running(void) {
	// TODO: Implement and get next state based on fsm diagram

	uint16_t bearing;
	double desired_heading, error;
	stateId_t next_state;
	EC_StatusTypeDef ec_status;
	CTRL_StatusTypeDef ctrl_status;

	ctrl_status = heading_received();


	if (ctrl_status != CTRL_OK) return ERROR;

	desired_heading = get_desired_heading(); //from base library//do error check

	ec_status = ecompass_getBearing(&bearing);
	
	if (ec_status != EC_OK) return ERROR;
	
	error = desired_heading - bearing;;

	if (error >= SMALL_ANGLE){ //make heading_received() a function in controls.c/h
			main_ctrl_loop();
			next_state = STATE_RUNNING;
	}

	else {  //error > small_angle
			motor_turn_off();
			next_state = STATE_IDLE;
	}
		

	return next_state;
}

stateId_t error(void) {
	// TODO: Implement and get next state based on fsm diagram
	uint16_t bearing;
	double desired_heading, error;
	stateId_t next_state;
	double wait_time = 0; //type may need to change

	//placeholder variable type
	ERROR_StatusTypeDef error_status;

	error_status = read_error_status();

	if (error_status == E_SOLVED){
		//im assuming here that software will send some kind of message through CAN indicating error status

		motor_turn_on();
		next_state = STATE_RUNNING;
		return next_state;
	}

	else{//if error couldn't be solved
		motor_turn_off();

		while (wait_time < MAX_WAIT){
			wait_time = timer();//start some kind of timer that tracks time elapsed since error encountered

			if (error_status == E_SOLVED){
				motor_turn_on();
				next_state = STATE_RUNNING;

				reset_timer(); //implement

				return next_state;
			}
		}

		NVIC_SystemReset(); //apparently this resets the stm32, need to test it out at some point
	}

}

