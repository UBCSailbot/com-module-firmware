/*
 *  error.h
 *
 *  Description: Provides declarations for variables and function prototypes related to error handling.
 *
 *  Created on: Mar 30, 2024
 *  Author: Moiz
 */
#include "board.h"

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

#define ERROR_LOG_SIZE 10

typedef enum
{
	ERR_LOW,
	ERR_MID,
	ERR_HIGH
}errlevel;

typedef enum
{
	SLOW_BLINK,
	FAST_BLINK,
}ledpattern;

typedef enum
{
	OK,
	ERR
} errcode;

// TODO: Move both of these definitions to source code later otherwise the compiler will complain (multiple definitions)
errcode errlog[ERROR_LOG_SIZE];
extern uint8_t index;

//-- Internal functions
void reportError(errcode err_code, errlevel err_level); // Sending all accumulated error codes through UART and/or through CAN
void logError(errcode err_code); // Add error to error logs. Can be stored in memory
void ledIndicationError(ledpattern pattern);
void I2C_ErrorResetCycle(I2C_HandleTypeDef handle, uint16_t device_address, uint8_t register_address, int read_regs);
void UART_Error_Handler(UART_HandleTypeDef *huart, uint8_t *pData);

//-- Public Functions
#define REPORT_ERR(err_code, err_level) (reportError(err_code, err_level))
errcode getLatestError(void);
void clearAllErrors(void);

#endif /* INC_ERROR_H_ */
