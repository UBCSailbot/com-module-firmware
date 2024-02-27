/*
 * unit_tests.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#ifndef SRC_UNIT_TESTS_H_
#define SRC_UNIT_TESTS_H_

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
		unsigned int raw;
		unsigned short seg[2];
	} error;
} testresult;

typedef enum {
	ALL,
	ECOMPASS
} testgroup;

typedef struct {
	char * testname;
	testresult (*func)(void);
	testgroup group;
}t_test;


void test_main(void);

#endif /* SRC_UNIT_TESTS_H_ */
