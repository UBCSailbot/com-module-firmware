/*
 * 	debug.c
 *
 * 	Description: Provides debugging functions such as printing/storing logs, key mapping, etc.
 *
 *  Created on: Mar 29, 2024
 *	Author: Peter, Jordan, Katherine
 */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"

/* Variables ------------------------------------------------------------------*/
uint8_t UART1_rxBuffer[1] = {0};

/* Debugging ------------------------------------------------------------------*/

/* Keyboard Mapping:
 * Takes in input from the serial monitor and returns ASCII int value.
 * Refer to the table: https://www.cs.cmu.edu/~pattis/15-1XX/common/handouts/ascii.html
 */
int debug_key(void){
	pwm3_init_ch1(50);

	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);

	return *UART1_rxBuffer;
}
