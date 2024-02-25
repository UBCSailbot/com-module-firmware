/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#include "unit_tests.h"

void test_main(void) {

	// Go through test runnner
	for (int i = 0; i < sizeof(test_runner)/sizeof(t_test); ++i) {
		const t_test test = test_runner[i];

		//printf("Running test: %s \r\n", test.testname);
		testresult result = test.func();
	}
}


testresult temp_test1(void) {
	testresult res = {TSUCCESS, 0};
	return res;
}

