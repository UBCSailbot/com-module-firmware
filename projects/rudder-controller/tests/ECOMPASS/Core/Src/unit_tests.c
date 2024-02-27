/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */

#include "unit_tests.h"
#include <stdio.h>

#define NUM_ITERS 10

const testgroup TEST_GROUP_SEL = ALL;

// Test functions
testresult temp_test1(void);
testresult iteration_test(void);

t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Temporary test 1", .func=temp_test1, .group=ECOMPASS},
		{.testname="Iteration test", .func=iteration_test}
};


void test_main(void) {
	unsigned int pass_count = 0;
	unsigned int num_tests = sizeof(test_runner)/sizeof(t_test);

	// Go through test runnner
	for (int i = 0; i < num_tests; ++i) {
		const t_test test = test_runner[i];

		if (TEST_GROUP_SEL == ALL || TEST_GROUP_SEL == test.group) {
			printf("Running test: %s \r\n", test.testname);
			testresult result = test.func();

			if (result.stat == TSUCCESS) pass_count++;
		}
	}

	printf("Tests Passed: %d \n\rTests Failed: %d", pass_count, num_tests - pass_count);
}


testresult temp_test1(void) {
	testresult res = {TSUCCESS, 0};
	return res;
}

testresult iteration_test(void) {
	testresult res = {TSUCCESS, 0};

	for (int i = 0; i < NUM_ITERS; ++i) {
		// res.error.raw = someFunction(..);
	}
	return res;
}

