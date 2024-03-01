/*
 * unit_tests.h
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#ifndef TEST_ENGINE_H_
#define TEST_ENGINE_H_

#include "uconfig.h"

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

typedef struct {
	char * testname;
	testresult (*func)(void);
	testgroup group;
}t_test;


void test_main(const t_test * test_runner, unsigned int size);

#endif /* SRC_UNIT_TESTS_H_ */
