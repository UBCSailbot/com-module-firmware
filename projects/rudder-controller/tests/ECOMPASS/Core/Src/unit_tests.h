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
}t_test;

void test_main(void);

// Test functions
testresult temp_test1(void);

t_test test_runner[] = {
		{.testname="Temporary test 1", .func=temp_test1}
};

#endif /* SRC_UNIT_TESTS_H_ */
