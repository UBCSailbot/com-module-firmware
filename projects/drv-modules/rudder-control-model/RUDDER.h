/*
 * This library implements a PID model with various extra functionality to account for dynamic sailing conditions
 * It currently has the following functionality:
 *		-TBD
 *
 *  Created on: Mar 26, 2025
 *      Author: Emma Duong
 */

#include "IMU.h"
#include "WINDSENSOR.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct {
	PIDcoefficients pidCoeffs;
	ScalingCoefficients scalingCoeffs;
	ControllerState controllerState;
} PIDController;

typedef struct {
	float Kp;
	float Kd;
	float Ki;
	float integralMax;
	float outputMax;
	float outputMin;
	float integralDecayFactor;
	float derivativeFilterFactor;
	float errorThreshold;
} PIDcoefficients;

typedef struct {
	float velocityFactor;
	float heelFactor;
} ScalingCoefficients;

typedef struct {
	float integralError;
	float previousError;
	float filteredError;
	float previousFilteredError;
	uint32_t lastTime;
	uint32_t currentTime;
} ControllerState;

enum State {
	STRAIGHT,
	TACKING,
	GYBING,
	LOWWIND,
	IRONS
};

// runs the PID controller and generates rudder angles to sail according to the desired heading
void runPID(PIDController *rudderController);

// initializes the PID controller parameters
void initPID(PIDController *rudderController, float Kp, float Kd,   float Ki, float integralMax, float outputMax, float outputMin, float integralDecayFactor, float derivativeFilterFactor, float errorThreshold);
