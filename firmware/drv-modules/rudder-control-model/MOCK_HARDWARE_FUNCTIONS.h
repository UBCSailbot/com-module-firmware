/* MOCK_HARDWARE_FUNCTIONS.h
 *  Created on: Oct 18, 2025
 *      Author: Emma Duong
 * This header file contains mock implementations of hardware-specific functions for compiler tests
 */
#ifndef MOCK_HARDWARE_H
#define MOCK_HARDWARE_H

#include <stdint.h>
#include <stdbool.h>

// === Time Functions (HAL) ===
// Mocks the microcontroller's system tick counter (used for dt calculations)
uint32_t HAL_GetTick(void);

// === Sensor/State Functions (Inputs) ===
// Mocks retrieving current sensor data for the controller

// Wind State
float getWindSpeed(void);
float getWindDirection(void);

// Sailing State (IMU/GPS data)
float getLinearVelocity(void);
float getAngularVelocity(void);
float getHeelAngle(void);
float getCurrentHeading(void);
float getDesiredHeading(void);

// === Control Output Functions (Not strictly needed for compilation, but good practice) ===
// (You did not show a function for setting the rudder, but it's likely needed)
void setRudderAngle(float angle);


#endif // MOCK_HARDWARE_H