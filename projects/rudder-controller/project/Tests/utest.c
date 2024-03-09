/*
 * unit_tests.c
 *
 *  Created on: Feb 24, 2024
 *      Author: Sailbot
 */
#include <stdio.h>
#include "utest.h"
#include "uconfig.h"
#include "ecompass.h"

// -- Test specific definitions
#define NUM_ITERS 20

const testgroup TEST_GROUP_SEL = ALL;

// -- Add to test runner here --
t_test test_runner[] = {
//		{"Name of test", "function definition", "testgroup id"
		{.testname="Read bearing value", .func=read_bearing_test, .group=ECOMPASS},

//pitch test
		{.testname = "Read pitch value", .func = pitch_test, .group=ECOMPASS}
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


testresult pitch_test(void){
	testresult res = {TSUCCESS, {0}};

	uint8_t pitch;

		for (int i = 0; i < NUM_ITERS; ++i) {
			res.error.raw = ecompass_getPitch(&pitch);
			if (res.error.raw != EC_OK) {
				res.stat = TERROR;
				return res;
			}

			printf("Pitch: %d\r\n", pitch);

			HAL_Delay(1000);
		}
		return res;
	}


void run_tests(void) {
	test_main(test_runner, sizeof(test_runner) / sizeof(t_test));
}




