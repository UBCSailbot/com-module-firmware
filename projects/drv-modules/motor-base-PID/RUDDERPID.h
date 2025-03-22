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
#include "stm32u5xx_hal.h  

// Constants
#define PROPORTIONAL_GAIN 0.0017f
#define INTEGRAL_GAIN 0.0000004f
#define ERROR_THRESHOLD 1.0f
#define INTEGRAL_LIMIT 15000
#define MOTOR_STOP 0
#define MAX_MOTOR 1.0f

// Function prototypes
void DAC_STEP(int step);
void Set_Motor(float Motor_Control);
void PI_Motor(int32_t desired_heading, int32_t current_heading,
              float* integral_error, uint32_t* last_time_stamp,
              int32_t* past_encoder_heading, int8_t* past_motor_direction);

#endif // MOTOR_PID_H
