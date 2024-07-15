/*
 *  error.h
 *
 *  Description: Provides declarations for variables and function prototypes related to error handling.
 *
 *  Created on: Mar 30, 2024
 *  Author: Moiz
 */

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
	ERROR
} errcode;

// TODO: Move both of these definitions to source code later otherwise the compiler will complain (multiple definitions)
errcode errlog[ERROR_LOG_SIZE];
uint8_t index;



//-- Internal functions
static void reportError(errcode err_code, errlevel err_level); // Sending all accumulated error codes through UART and/or through CAN
static void logError(errcode err_code); // Add error to error logs. Can be stored in memory
static void ledIndicationError(ledpattern pattern);

//-- Public Functions
#define REPORT_ERR(err_code, err_level) (reportError(err_code, err_level))
errcode getLatestError(void);
void clearAllErrors(void);

#endif /* INC_ERROR_H_ */
