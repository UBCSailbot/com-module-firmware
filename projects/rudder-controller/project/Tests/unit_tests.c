/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#include "unit_tests.h"
#include <stdio.h>
#include "ecompass.h"

const testgroup TEST_GROUP_SEL = ALL;

testresult read_bearing_test(void);

// -- Add to test runner here --
t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Read bearing value", .func=read_bearing_test, .group=ECOMPASS}
};

// -- Test specific definitions
#define NUM_ITERS 10


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

// -- Framework --
void test_main(void) {
	unsigned int pass_count = 0;
	unsigned int num_tests = sizeof(test_runner)/sizeof(t_test);

	// Go through test runnner
	for (int i = 0; i < num_tests; ++i) {
		const t_test test = test_runner[i];

		if (TEST_GROUP_SEL == ALL || TEST_GROUP_SEL == test.group) {
			printf("--- Running Test: %s --- \r\n", test.testname);
			testresult result = test.func();

			if (result.stat == TSUCCESS) {
				printf("PASS \r\n");
				pass_count++;
			} else {
				printf("FAIL (%2x)(%2x) \r\n", result.error.seg[1], result.error.seg[0]);
			}

		}
	}

	printf("--- Test Results --- \r\n");

	printf("Tests Passed: %d \n\rTests Failed: %d\n\r", pass_count, num_tests - pass_count);
}

