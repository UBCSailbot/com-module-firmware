/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#include "test_engine.h"
#include <stdio.h>

// -- Framework --
void test_main(const t_test *test_runner, unsigned int size) {
	unsigned int pass_count = 0;
	unsigned int num_tests = size;

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

