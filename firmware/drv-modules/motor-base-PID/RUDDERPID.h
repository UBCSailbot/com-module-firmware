/*
 * This library provides a PI controller using the microcontroller and Groschopp motor and motor controller.
 * It currently has the following functionality:
 *		-A method to test the minimum required power to drive the motor
 *		-A function to change speed and direction of the motor
 *		-A PI function that controls the motor to a desired heading
 *
 * For this library to work as intended, do the follwoing:
 * Pin A4 has to be enabled as DAC 1/CHANNEL 1 
 * PIN A7 has to be enabled as a GPIO output
 * USART 1 has to be enabled to print debug statements in PuTTY
 * 
 * The encoder hardware and circuit has to be setup
 * The encoder header file must be included in the src file
 * The encoder object has to be created in the main file
 *  Created on: Mar 18, 2025
 *      Author: Chukwudalu Joshua Obi
 */



#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "stm32u5xx_hal.h"

// Constants

#define PROPORTIONAL_GAIN 0.05//0.003	// Proportional gain tuning parameter
#define INTEGRAL_GAIN 0.001//0.0001		// Integral gain tuning parameter
#define ERROR_THRESHOLD 1.0f		// Desired error threshold
#define INTEGRAL_LIMIT 15000		// Set for integral clamp to prevent integral error from growing too large
#define MOTOR_STOP 0				// Zero output from DAC
#define MAX_MOTOR 1.0f				// Full output of the DAC
#define MAX_ANGLE 45.0f				// Maximum/minimum range of motion of the rudder
#define MIN_MOTOR 0.062				// Sets the motor dead band where 0 would be 0V of dead band and 1.0 would be 3.3V of dead band
#define MIN_MOTOR_VELOCITY_COMMAND 0.0001
#define DAC_MAX_OUTPUT 4095
#define HARD_ANGLE_LIMIT 50.0f		//Absolute maximum range

typedef struct {
	DAC_HandleTypeDef *motorDacPeripheral;
	uint32_t motorDacChannel;
	GPIO_TypeDef *enableGPIOPeripheral;
	uint16_t enableGPIOPin;
	GPIO_TypeDef *reverseGPIOPeripheral;
	uint16_t reverseGPIOPin;
 } MOTOR_CONFIG;

// Function prototypes
void Setup_Motor(MOTOR_CONFIG motorConfig);
void Enable_Motor();
void Disable_Motor();
void DAC_STEP(int step);
void Set_Motor_Raw(float Motor_Control);
void Set_Motor_Calibrated(float Motor_control);
void PI_Motor(float desired_heading, float current_heading, uint32_t angle_timestamp);

#endif // MOTOR_PID_H
