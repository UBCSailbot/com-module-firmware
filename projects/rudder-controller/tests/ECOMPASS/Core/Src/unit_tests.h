/*
 * unit_tests.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#ifndef SRC_UNIT_TESTS_H_
#define SRC_UNIT_TESTS_H_

//typedef enum {
//	TEMP_TEST1
//} testname;

typedef enum {
	COMPONENT_INVALIDREAD = 0x01
} errorcode;

typedef enum {
	TSUCCESS,
	TERROR
} teststatus;

typedef struct {
	teststatus stat;
	union Error {
		unsigned char raw;
		errorcode code;
	} error;
} testresult;

typedef struct {
	char * testname;
	testresult (*func)(void);
	testgroup group;
}t_test;

typedef enum {
	ALL,
	ECOMPASS
} testgroup;

void test_main(void);


#endif /* SRC_UNIT_TESTS_H_ */
