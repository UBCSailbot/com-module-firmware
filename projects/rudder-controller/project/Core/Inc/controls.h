/*
 * controls.h
 *
 *  Created on: Mar 16, 2024
 *      Author: Jasia
 */

#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#include "main.h"
#include <stddef.h>


typedef enum
{
	CTRL_OK = 0x00,
	CTRL_ERROR = 0x01
} CTRL_StatusTypeDef;

void run_pid(void);
void main_ctrl_loop(void);

//watchdog timer- for future reference

#endif /* INC_CONTROLS_H_ */
