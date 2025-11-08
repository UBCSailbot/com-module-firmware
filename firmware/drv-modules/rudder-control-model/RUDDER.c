/* RUDDER.c
 *  Created on: Mar 26, 2025
 *      Author: Emma Duong
 *
 *  This file contains the implementation of a PID controller for Polaris' rudder.
 *  The controller adjusts the rudder angle based on the boat's current state, wind conditions,
 *  and desired heading.
 *
 *  The controller operates in different modes depending on the sailing state:
 *  - Straight Line Sailing
 *  - Tacking
 *  - Gybing
 *  - Low Wind Conditions
 *  - Irons (stuck facing into the wind)
 *
 *  Each mode has its own set of PID coefficients and scaling factors to adapt to the specific dynamics of that state.
 *  The controller continuously updates its state based on real-time data from wind sensors and IMU readings.
 *
 *  Key Functions:
 *  - initPID: Initializes the PID controller with fixed parameters.
 *  - updateControllerVariables: Updates the controller's live variables with current sensor data.
 *  - getRudderAngle: Computes the new rudder angle based on the current error and PID calculations.
 *  - runPID: Main function to determine the current sailing state and apply the appropriate control logic.
 *  - isTackingCondition & isGybingCondition: Determine if the boat should enter tacking or gybing states based on wind and heading.
 *
 *  Notes: 
 *  - This implementation assumes access to certain hardware-specific functions for retrieving sensor data,
 *        which are currently placeholders given that some of the actual hardware has not been confirmed nor libraries written
 *  - This code is untested, coefficients and thresholds are placeholders * 
 *
 *  Future Improvements:
 *         -Tuning of PID coefficients for each sailing state
 *       -Integration with other boat systems (e.g., sail control)
 *      -Enhanced state detection algorithms
 *     -Robustness against sensor noise and failures
 */

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
//#include "MOCK_HARDWARE_FUNCTIONS.h"

#define WINDOW_SIZE 10

// Global controller instance
PIDController controller;

/*Private function declarations*/

static void updateAverages(void);
static float movingAverage(float array[], int index, int size, float value);
static float straightLine(float error);
static float tacking(void);
static float gybing(void);
static float lowwind(float error);
static float irons(float error);
static bool isTackingCondition(float error);
static bool isGybingCondition(float error);

// Initializes the live state of the PID controller
PIDControllerLive initLiveController() {
    PIDControllerLive live;
    memset(&live, 0, sizeof(PIDControllerLive)); // Zero out the struct
    // initialize controller state - time, integral, etc
    ControllerState cState;
    cState.lastTime  = HAL_GetTick();
    cState.currentTime = cState.lastTime;
    // get wind data and initialize wind state
    volatile WindState wState;
    // wState.windSpeed = getWindSpeed();
    // wState.windDirection = getWindDirection();
    // initialize sailing state and get IMU data
    volatile SailingState sState;
    #ifdef STRAIGHT_ONLY
    sState.angularVelocity = 0.0f;
    #endif
    // sState.linearVelocity = getLinearVelocity();
    // sState.angularVelocity = getAngularVelocity();
    // sState.heelAngle = getHeelAngle();
    // sState.currentHeading = getCurrentHeading();
    // sState.desiredHeading = getDesiredHeading();
    sState.averageLinVelocity = sState.linearVelocity;
    sState.averageAngVelocity = sState.angularVelocity;
    // sState.averageHeading = sState.currentHeading;
    // initialize tacking state
    TackingState tState;
    tState.isTacking = false;
    live.tackingState = tState;
    // initialize gybing state
    GybingState gState;
    gState.isGybing = false;
    live.gybingState = gState;
    // assign states to live controller
    LiveValues liveVals;

    PIDcoefficients activeCoeffs;

    live.activeCoeffs = activeCoeffs;
    live.controllerState = cState;
    live.windState = wState;
    live.sailingState = sState;
    live.tackingState = tState;
    live.gybingState = gState;
    live.liveValues = liveVals;

    return live;
}

// Initializes PID controller parameters
void initController(PIDControllerFixed fixedController) {
    PIDControllerLive *live = &controller.live;
    *live = initLiveController();
    controller.fixed = fixedController;
}

void updateControllerVariables () {
    // Update the sailing and sea state in the controller
//    controller.live.windState.windSpeed = getWindSpeed();
//    controller.live.windState.windDirection = getWindDirection();
//    controller.live.sailingState.linearVelocity = getLinearVelocity();
//    controller.live.sailingState.angularVelocity = getAngularVelocity();
//    controller.live.sailingState.heelAngle = getHeelAngle();
//    controller.live.sailingState.currentHeading = getCurrentHeading();
//    controller.live.sailingState.desiredHeading = getDesiredHeading();
    updateAverages();
}

void updateAverages() {
    // Simple moving average for linear velocity, angular velocity, and heading
    static float linVelocityBuffer[WINDOW_SIZE] = {0.0f};
    static float angVelocityBuffer[WINDOW_SIZE] = {0.0f};
    static float headingBuffer[WINDOW_SIZE] = {0.0f};
    static int index = 0;
    static int count = 0;
    index = (index + 1) % 10;
    if(count < 10) {
        count++;
    }
    controller.live.sailingState.averageLinVelocity = movingAverage(linVelocityBuffer, index, count, controller.live.sailingState.linearVelocity);
    controller.live.sailingState.averageAngVelocity = movingAverage(angVelocityBuffer, index, count, controller.live.sailingState.angularVelocity);
    controller.live.sailingState.averageHeading = movingAverage(headingBuffer, index, count, controller.live.sailingState.currentHeading);
}

float movingAverage(float array[], int index, int size, float value) {
    array[index] = value;
    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += array[i];
    }
    return sum / size;
}

void resetController() {
    memset(&controller.live, 0, sizeof(PIDControllerLive));
    // Reset the controller time
    controller.live.controllerState.lastTime  = HAL_GetTick();
    controller.live.controllerState.currentTime = controller.live.controllerState.lastTime;
    // Reset active coefficients to standard
    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
}

// Gets a new rudder angle based on the current error
float getRudderAngle(float currentError) {

    #ifdef TUNING_MODE
    volatile PIDcoefficients *PID = &controller.fixed.standardCoeffs;
    #else
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    #endif
    ControllerState *cState = &controller.live.controllerState;
    ScalingCoefficients *scaling = &controller.fixed.scalingCoeffs;
    PhysicalParams *params = &controller.fixed.physicalParams;
    volatile SailingState *sailing = &controller.live.sailingState;
    LiveValues *vals = &controller.live.liveValues;

    vals->errorValue = currentError;

	cState->currentTime = HAL_GetTick();
	float dt = cState->currentTime -  cState->lastTime;
    // Calculating the integral addition
    float integral = cState->integralError + currentError * dt;
    // Clamp integral to prevent windup
    const float integralMax = PID->integralMax; // from your PhysicalParams or a #define
    if (integral > integralMax) {
        integral = integralMax;
    } else if (integral < -integralMax) {
        integral = -integralMax;
    }

    vals->integralValue = integral;

    // Save the clamped integral back
    cState->integralError = integral;
    // Low pass filtering the derivative
    cState->filteredError = PID->derivativeFilterFactor * currentError + (1 - PID->derivativeFilterFactor) * cState->previousFilteredError;
    // Calculating the derivative
    // Handling division by zero/ negative time
    float derivative;
    if(dt <= 0) {
        derivative = 0;
    } else {
        derivative = (cState->filteredError - cState->previousFilteredError) / dt;
    }   
    vals->derivativeValue = derivative; 
    // Calculating PID output
    float outputAngle = PID->Kp * currentError + PID->Ki * integral + PID->Kd * derivative;
    // Saving error
    cState->previousFilteredError = cState->filteredError;
    // Scaling and clamping output and integral  
    
    #ifdef TUNING_MODE
    #else
    if(sailing->averageLinVelocity > 0) {
        outputAngle = (scaling->velocityFactor/pow(sailing->averageLinVelocity, 2.0))*(1-scaling->heelFactor*sailing->heelAngle)*outputAngle;
    }
    #endif
        // limit to max rudder angle
    if(outputAngle > params->outputMax) {
        outputAngle = params->outputMax;
        if(currentError < 0) {
            cState->integralError += currentError * dt; 
         }

    } else if(outputAngle < params->outputMin) {
        outputAngle = params->outputMin;
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
    return outputAngle;
}

// Updates the controller state without generating new output
void updateControllerTime(float currentError) {
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    ControllerState *cState = &controller.live.controllerState;
    cState->lastTime = cState->currentTime;
	cState->currentTime = HAL_GetTick();
    cState->previousError = currentError;    
    cState->integralError = cState->integralError * PID->integralDecayFactor;
}

// Runs control model based on current sailing state
void runPID(float *rudderAngle) {

    volatile SailingState *sailing = &controller.live.sailingState;

    updateControllerVariables();

    float error = sailing->currentHeading - sailing->desiredHeading;
    // Normalize error to be within -180 to 180 degrees
    if(error > 180){
        error -= 360;
    }
    else if(error < -180){
        error += 360;
    }
    
    State state = getState(error);

	switch(state){
	case STRAIGHT:
		*rudderAngle = straightLine(error);
        break;
	case TACKING:
		*rudderAngle = tacking();
		break;
	case GYBING:
		*rudderAngle = gybing();
		break;
	case LOWWIND:
		*rudderAngle = lowwind(error);
		break;
	case IRONS:
		*rudderAngle = irons(error);
		break;
	}    
}

State getState(float error) {
    PhysicalParams *params = &controller.fixed.physicalParams;    
    volatile SailingState *sailing = &controller.live.sailingState;
    volatile WindState *wind = &controller.live.windState;
    StateThresholds *thresholds = &controller.fixed.stateThresholds;
    
    #ifdef STRAIGHT_ONLY
    return STRAIGHT;
    #endif

    if (isTackingCondition(error)) {
        return TACKING;
    } else if (isGybingCondition(error)) {
        return GYBING;
    } else if (wind->windSpeed < params->lowWindThreshold) {
        return LOWWIND;
    } else if (sailing->angularVelocity < thresholds->ironsSpeed && sailing->angularVelocity < thresholds->stateironsRot && 
        (fabs(wind->windDirection - sailing->currentHeading) < params->upwindIronsAngle || fabs(wind->windDirection - sailing->currentHeading) < params->downwindIronsAngle)) {
        return IRONS;
    } else {
        return STRAIGHT; // Default to straight if no other conditions met
    }
}

//To do, check if these conditions are correct - sign conventions, etc.
bool isTackingCondition(float error) {
    volatile WindState *wind = &controller.live.windState;
    volatile SailingState *sailing = &controller.live.sailingState;

    float relativeWind = wind->windDirection - sailing->desiredHeading;
    float boatWindAngle = wind->windDirection - sailing->currentHeading;

    if(controller.live.tackingState.isTacking) {
        // If already tacking, continue until duration is over
        if(HAL_GetTick() - controller.live.tackingState.tackingStartTime < controller.fixed.scalingCoeffs.tackTime) {
            return true;
        } else {
            controller.live.tackingState.isTacking = false;
            return false;
        }
    } else if(fabs(boatWindAngle) < 90 || fabs(boatWindAngle) > 270) {
        // Check if the desired heading is on the opposite side of the wind direction
        if((relativeWind > 0 && boatWindAngle < 0) || (relativeWind < 0 && boatWindAngle > 0)){
            return true;
        }
    }
    return false;    
}

//To do, check if these conditions are correct - sign conventions, etc.
bool isGybingCondition(float error) {
    volatile  WindState *wind = &controller.live.windState;
    volatile SailingState *sailing = &controller.live.sailingState;

    float relativeWind = wind->windDirection - sailing->desiredHeading;
    float boatWindAngle = wind->windDirection - sailing->currentHeading;

    if(controller.live.gybingState.isGybing) {
        // If already gybing, continue until duration is over
        if(HAL_GetTick() - controller.live.gybingState.gybingStartTime < controller.fixed.scalingCoeffs.gybeTime) {
            return true;
        } else {
            controller.live.gybingState.isGybing = false;
            return false;
        }
    } else
    if(fabs(boatWindAngle) > 90 && fabs(boatWindAngle) < 270) {
        if((relativeWind > 0 && boatWindAngle < 0) || (relativeWind < 0 && boatWindAngle > 0)){
            return true;
        }
    }
    return false;    
}

// Control model for straight line sailing
//TODO: Change so that the model returns to below our heading tolerance versus oscillating around the error threshold
float straightLine(float error) {
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    volatile SailingState *sailing = &controller.live.sailingState;
    ControllerState *cState = &controller.live.controllerState;

    controller.live.activeCoeffs = controller.fixed.standardCoeffs;

    float rudderAngle;
	
    // if within error threshold, do not adjust rudder angle
	if(fabs(error) < PID->errorThreshold && !cState->isActive) {
        // update controller state without changing output
		updateControllerTime(error);
        rudderAngle = 0;
	}
	else {
        cState->isActive = true;
        // compute scaled angle from PID output
        rudderAngle = getRudderAngle(error);
        // if within heading tolerance, stop adjusting rudder angle
        if(fabs(sailing->averageHeading - sailing->desiredHeading) < PID->headingTolerance && fabs(sailing->averageAngVelocity) < PID->angVelTolerance) {
            cState->isActive = false;
        }
	}
    return rudderAngle;
}

//Control model for tacking maneuvers
float tacking() {
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    volatile SailingState *sailing = &controller.live.sailingState;
    TackingState *tacking = &controller.live.tackingState;
    ScalingCoefficients *scaling = &controller.fixed.scalingCoeffs;
    StateThresholds *thresholds = &controller.fixed.stateThresholds;

    // If not already tacking, initialize tacking state with target heading and duration
    if(!tacking->isTacking) {
        tacking->isTacking = true;
        tacking->tackingStartTime = HAL_GetTick();
        tacking->initialHeading = sailing->currentHeading;

        if(tacking->initialHeading - sailing->desiredHeading > 0) {
        // Tack to starboard
            tacking->targetHeading = sailing->desiredHeading + scaling->tackHeadingPadding;
        } else {
        // Tack to port
            tacking->targetHeading = sailing->desiredHeading - scaling->tackHeadingPadding;
        }   
    // Normalize target heading to be within 0-360 degrees
        if(tacking->targetHeading > 360) {
            tacking->targetHeading -= 360;
        } else if(tacking->targetHeading < 0) {
            tacking->targetHeading += 360;
        }

        *PID = controller.fixed.tackingCoeffs;
        return getRudderAngle(sailing->currentHeading - tacking->targetHeading);

    } else {
        // If already tacking, check if conditions to end tack are met (tacking duration over or close to target heading)
        if(sailing->averageHeading > PID->headingTolerance && sailing->averageAngVelocity > thresholds->tackingRotThreshold) {
            float error = sailing->currentHeading - tacking->targetHeading;
            // Normalize error to be within -180 to 180 degrees
            if(error > 180){
                error -= 360;
            }
            else if(error < -180){
                error += 360;
    }
            return getRudderAngle(error);
        } else {
            tacking->isTacking = false;
            *PID = controller.fixed.standardCoeffs;
            return 0.0f; // End of tack, return rudder to neutral
        }      
    }
}

float gybing() {
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    volatile SailingState *sailing = &controller.live.sailingState;
    GybingState *gybing = &controller.live.gybingState;
    ScalingCoefficients *scaling = &controller.fixed.scalingCoeffs;
    StateThresholds *thresholds = &controller.fixed.stateThresholds;

    // If not already gybing, initialize gybing state with target heading and duration
    if(!gybing->isGybing) {
        gybing->isGybing = true;
        gybing->gybingStartTime = HAL_GetTick();
        gybing->initialHeading = sailing->currentHeading;

        if(gybing->initialHeading - sailing->desiredHeading > 0) {
        // Gybe to starboard
            gybing->targetHeading = sailing->desiredHeading + scaling->gybeHeadingPadding;
        } else {
        // Gybe to port
            gybing->targetHeading = sailing->desiredHeading - scaling->gybeHeadingPadding;
        }
    // Normalize target heading to be within 0-360 degrees
        if(gybing->targetHeading > 360) {
            gybing->targetHeading -= 360;
        } else if(gybing->targetHeading < 0) {
            gybing->targetHeading += 360;
        }
        *PID = controller.fixed.gybingCoeffs;
        return getRudderAngle(sailing->currentHeading - gybing->targetHeading);
    } else {
        // If already gybing, check if conditions to end gybe are met (gybing
        if(sailing->averageHeading > PID->headingTolerance && sailing->averageAngVelocity > thresholds->gybingRotThreshold) {
            float error = sailing->currentHeading - gybing->targetHeading;
            // Normalize error to be within -180 to 180 degrees
            if(error > 180){
                error -= 360;
            }
            else if(error < -180){
                error += 360;
            }
            return getRudderAngle(error);
        } else {
            gybing->isGybing = false;
            *PID = controller.fixed.standardCoeffs;
            return 0; // End of gybe, return rudder to neutral
        }
    }
}

//low wind control model
//TODO: Change so that the model returns to below our heading tolerance versus oscillating around the error threshold
float lowwind(float error) {
    PIDcoefficients *PID = &controller.live.activeCoeffs;
    volatile SailingState *sailing = &controller.live.sailingState;
    ControllerState *cState = &controller.live.controllerState;

    *PID = controller.fixed.lowWindCoeffs;

    float rudderAngle;
    
    // if within error threshold, do not adjust rudder angle
    if(fabs(error) < PID->errorThreshold && !cState->isActive) {
        // update controller state without changing output
        updateControllerTime(error);
        rudderAngle = 0;
    }
    else {
        cState->isActive = true;
        rudderAngle = getRudderAngle(error);
        // if within heading tolerance, stop adjusting rudder angle
        if(fabs(sailing->averageHeading - sailing->desiredHeading) < PID->headingTolerance && fabs(sailing->averageAngVelocity) < PID->angVelTolerance) {
            cState->isActive = false;
        }
    }
    return rudderAngle;
}

//TODO: Implement this - kind of error handling for entire model
float irons(float error) {
	return 0.0;
}







