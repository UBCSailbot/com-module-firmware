/* main_test.c
 *  Created on: Oct 18, 2025
 *      Author: Emma Duong
 * This is a simple test harness to compile and run the PID controller
 */

#include <stdio.h>
#include "RUDDER.h"          // Contains function prototypes for your controller
#include "RUDDER_PARAMS.h"   // Contains definitions for fixed struct types
#include "MOCK_HARDWARE_FUNCTIONS.h"   // Contains declarations for your mock functions


int main() {
    printf("Starting PID Controller Mock Test...\n");

    // 1. Get the fixed parameters (must be defined somewhere)
    PIDControllerFixed fixed_params = getRudderFixedParams();

    // 2. Initialize the global controller variable
    initController(fixed_params);

    float rudder_angle = 0.0f;
    int loop_count = 0;

    // 3. Run the controller loop for a few iterations
    while (loop_count < 20) {
        // This simulates the main execution loop in an embedded system
        runPID(&rudder_angle); 
        
        // Use the output function defined in MOCK_HARDWARE.c
        setRudderAngle(rudder_angle); 
        
        // Small delay simulation, as HAL_GetTick() increases in mock
        // For a desktop test, you might add a sleep here, but for now, the mock time is fine.

        loop_count++;
    }

    printf("Test finished after 20 loops.\n");
    return 0;
}