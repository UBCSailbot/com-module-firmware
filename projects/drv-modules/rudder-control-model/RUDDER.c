/*
 * RUDDER.c
 *
 *  Created on: Mar 26, 2025
 *      Author: Emma Duong
 */

#include "RUDDER.h"
#include "IMU.h"
#include "WINDSENSOR.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

/* wind speed in m/s */
float windSpeed;
/* wind direction in degrees (measured as a compass bearing) */
float windDirection;
/* linear velocity of boat in m/s */
float linearVelocity;
/* angular velocity in degrees/s */
float angularVelocity;
/* heel angle in degrees */
float heelAngle;
/* desired heading in degrees */
float desiredHeading;
/* current heading in degrees */
float currentHeading;
/* output rudder angle */
float desiredRudderAngle;

// Initializes PID controller parameters
void initPID(PIDController *rudderController) {
    ControllerState *cState = rudderController->controllerState;
    cState->integralError = 0;
    cState->previousError = 0;
    cState->filteredError = 0;
    cState->previousFilteredError = 0;
    cState-> lastTime  = HAL_GetTick();
    cState->currentTime = cState->lastTime;
}

// Gets a new rudder angle based on the current error
void getRudderAngle(PIDController *rudderController, float currentError) {
    PIDcoefficients *PID = rudderController->pidCoeffs;
    ControllerState *cState = rudderController->controllerState;
    ScalingCoefficients *scaling = rudderController->scalingCoeffs;

	cState->currentTime = HAL_GetTick();
	dt = cState->currentTime -  cState->lastTime;
    // Calculating the integral addition
    float integral = currentError * dt;
    // Low pass filtering the derivative
    cState->filteredError = PID->derivativeFilterFactor * currentError + (1 - PID->derivativeFilterFactor) * cState->previousFilteredError;
    // Calculating the derivative
    float derivative = (cState->filteredError - cState->previousFilteredError) / dt;
    // Calculating PID output
    float outputAngle = PID->Kp * currentError + PID->Ki * integral + PID->Kd * derivative;
    // Saving error
    cState->previousFilteredError = cState->filteredError;
    // Clamping output and integral to prevent windup
    if (outputAngle > PID->outputMax) {
        outputAngle = PID->outputMax;
        if(currentError < 0) {
            cState->integralError += currentError * dt; 
        }
    } else if (outputAngle < PID->outputMin) {
        outputAngle = PID->outputMin;
        if(currentError > 0) {
            cState->integralError += currentError * dt; 
        }
    } else {
        cState->integralError += currentError * dt;
    }
    // Apply integral decay
    cState->integralError *= PID->integralDecayFactor;
    cState->previousError = currentError;
    cState->lastTime = cState->currentTime;
    desiredRudderAngle = outputAngle;
}

// Updates the controller state without generating new output
void updateController(PIDController *rudderController, float currentError) {
    PIDcoefficients *PID = rudderController->pidCoeffs;
    ControllerState *cState = rudderController->controllerState;

	cState->currentTime = HAL_GetTick();
    cState->previousError = currentError;
    cState->lastTime = cState->currentTime;
    cState->integralError = cState->integralError * PID->integralDecayFactor;
}

// Runs control model based on current sailing state
void runPID(PIDController *rudderController){
    float error = currentHeading - desiredHeading;
    // Normalize error to be within -180 to 180 degrees
    if(error > 180){
        error -= 360;
    }
    else if(error < -180){
        error += 360;
    }

	switch(state){
	case STRAIGHT:
		straightLine(*rudderController, error);
		break;
	case TACKING:
		tacking(*rudderController, error);
		break;
	case GYBING:
		gybing(*rudderController, error);
		break;
	case LOWWIND:
		lowwind(*rudderController, error);
		break;
	case IRONS:
		irons(*rudderController, error);
		break;
	}
}

// Control model for straight line sailing
void straightLine(PIDController *rudderController, float error) {
    PIDcoefficients *PID = rudderController->pidCoeffs;
    ScalingCoefficients *scaling = rudderController->scalingCoeffs;
	
    // if within error threshold, do not adjust rudder angle
	if(abs(error) < ERRORTHRESHOLD){
		updateControllerTime(*rudderController, error);
        desiredRudderAngle = 0;
	}
	else {
        // compute scaled angle from PID output
		desiredRudderAngle = (scaling->velocityFactor/pow(linearVelocity, 2.0))*(1-scaling->heelFactor*heelAngle)*getRudderAngle(*rudderController, error);
        // limit to max rudder angle
        if(desiredRudderAngle > PID->outputMax){
            desiredRudderAngle = PID->outputMax;
        }
        else if(desiredRudderAngle < PID->outputMin){
            desiredRudderAngle = PID->outputMin;
        }
	}
}


void tacking(PIDController *rudderController) {
    
}

void gybing(PIDController *rudderController) {
}

void lowwind(PIDController *rudderController) {
}

void irons(PIDController *rudderController) {
}





