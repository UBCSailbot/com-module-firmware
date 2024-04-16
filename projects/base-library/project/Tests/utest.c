/*
 * utest.c
 *
 *  Created on: Feb 27, 2024
 *      Author: mlokh
 */

#include "utest.h"
#include "uconfig.h"


//-- Add tests to runner and custom test macros/definitions
#define NUM_ITERS 10

// -- Add to test runner here --
const t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Temporary test 1", .func=temp_test1, .group=TEMP},
		{.testname="Iteration test", .func=iteration_test}
};


// -- Unit tests --
testresult temp_test1(void) {
	testresult res = {TSUCCESS, {0}};
	return res;
}

testresult iteration_test(void) {
	testresult res = {TSUCCESS, {0}};

	for (int i = 0; i < NUM_ITERS; ++i) {
		// res.error.raw = someFunction(..);
	}
	return res;
}

void run_tests(void) {
	test_main(test_runner, sizeof(test_runner) / sizeof(t_test));
}



