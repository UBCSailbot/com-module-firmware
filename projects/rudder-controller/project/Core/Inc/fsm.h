/*
 * fsm.h
 *
 *  Created on: Mar 22, 2024
 *      Author: mlokh
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_


typedef enum {
	STATE_IDLE, //value of 0 automatically since nothing specified
	STATE_RUNNING, //1
	STATE_ERROR//2
} stateId_t;

// Store state variables
//this is the 'type' for the statemachine
//stores current state, error, etc
typedef struct {
	stateId_t current;
	struct motor {
		unsigned char on;
	}motor;
	unsigned int error;
}state_t;

stateId_t idle(void);
stateId_t running(void);
stateId_t error(void);
//each of these functions have associated motor, on, error and current parameters

stateId_t (*statefunc[])(void) = {
		//the (void) indicates that the functions in the statefunc array dont take any parameters
		idle, //value 0
		running, //1
		error //2
};

#endif /* INC_FSM_H_ */
