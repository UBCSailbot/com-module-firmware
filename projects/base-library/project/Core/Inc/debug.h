/*
 * 	debug.h
 *
 * 	Description: Provides declarations for variables and function prototypes related to debugging features
 *
 *  Created on: Mar 29, 2024
 *	Author: Peter, Jordan, Katherine
 */

#ifndef INC_OPRT_H_
#define INC_OPRT_H_

/* Includes ------------------------------------------------------------------*/
#include "board.h"

/* Variables ------------------------------------------------------------------*/
extern uint8_t UART1_rxBuffer[1];

/* Function prototypes ------------------------------------------------------------------*/
int debug_key(void);

#endif /* INC_OPRT_H_ */
