/*
 * fsm.h
 *
 *  Created on: Mar 22, 2024
 *      Author: mlokh
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_


typedef enum {
	STATE_IDLE,
	STATE_RUNNING,
	STATE_ERROR
} stateId_t;

// Store state variables
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

stateId_t (*statefunc[])(void) = {
		idle,
		running,
		error
};

#endif /* INC_FSM_H_ */
