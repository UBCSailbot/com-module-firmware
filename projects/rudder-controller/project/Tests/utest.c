/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */
#include <stdio.h>
#include "utest.h"
#include "ecompass.h"

// -- Test specific definitions
#define NUM_ITERS 10

const testgroup TEST_GROUP_SEL = ALL;

// -- Add to test runner here --
t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Read bearing value", .func=read_bearing_test, .group=ECOMPASS}
};


// -- Unit tests --
testresult read_bearing_test(void) {
	testresult res = {TSUCCESS, {0}};
	uint16_t bearing;

	for (int i = 0; i < NUM_ITERS; ++i) {
		res.error.raw = ecompass_getBearing(&bearing);
		if (res.error.raw != EC_OK) {
			res.stat = TERROR;
			return res;
		}

		printf("Bearing: %d\r\n", bearing);

		HAL_Delay(1000);
	}
	return res;
}

void run_tests(void) {
	test_main(test_runner, sizeof(test_runner) / sizeof(t_test));
}

