/* MOCK_HARDWARE_FUNCTIONS.c
 *  Created on: Oct 18, 2025
 *      Author: Emma Duong
 * 
 * This file contains mock implementations of hardware-specific functions for compiler tests
 */

#include "MOCK_HARDWARE_FUNCTIONS.h"
#include <stdio.h>

// A simple counter to simulate time passing for HAL_GetTick
static uint32_t mock_tick_count = 0;

// === Time Functions (HAL) ===

// Mocks the microcontroller's system tick counter (e.g., in milliseconds)
uint32_t HAL_GetTick(void) {
    // Increment the tick count to simulate the passage of time
    mock_tick_count += 20; // Simulate a 20ms update interval (50Hz)
    return mock_tick_count;
}

// === Sensor/State Functions (Inputs) ===

// Mocks current wind speed (m/s)
float getWindSpeed(void) {
    return 5.0f; // Placeholder: 5 m/s
}

// Mocks current wind direction (degrees from North)
float getWindDirection(void) {
    return 45.0f; // Placeholder: Northeast
}

// Mocks current over-water velocity (m/s)
float getLinearVelocity(void) {
    return 2.5f; // Placeholder: 2.5 m/s
}

// Mocks current rotational velocity (rad/s)
float getAngularVelocity(void) {
    return 0.0f; // Placeholder: Straight sailing
}

// Mocks current heel angle (degrees)
float getHeelAngle(void) {
    return 10.0f; // Placeholder: 10 degrees heeled
}

// Mocks current heading (degrees from North)
float getCurrentHeading(void) {
    return 20.0f; // Placeholder: Current heading
}

// Mocks desired heading (degrees from North)
float getDesiredHeading(void) {
    return 25.0f; // Placeholder: Desired heading
}

// === Control Output Functions ===

// Mocks the function that would command the hardware to move the rudder
void setRudderAngle(float angle) {
    // In a real application, this sends a signal to a servo/actuator.
    // Here, we just print the result for testing/debugging.
    printf("Rudder commanded to: %.2f degrees\n", angle);
}