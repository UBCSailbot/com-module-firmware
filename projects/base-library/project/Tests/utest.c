/*
 * utest.c
 *
 *  Created on: Feb 27, 2024
 *      Author: mlokh
 */
#include "utest.h"
#include "uconfig.h"
#include "bord.h"

//-- Add tests to runner and custom test macros/definitions
#define NUM_ITERS 10

// -- Add to test runner here --
const t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="UART print test", .func=UART_print_test, .group=UART}
};


// -- Unit tests --

testresult UART_print_test(void) {
	testresult res = {TSUCCESS, {0}};
	printf("hello world\n\r");

	return res;
}


void run_tests(void) {
	test_main(test_runner, sizeof(test_runner) / sizeof(t_test));
}


