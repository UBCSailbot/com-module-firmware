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
 *  Functions:
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
#include <stdio.h>
#include <stdbool.h>
#include "RUDDER.h"

/* This struct represents the live state of the PID controller
 * including all values that change dynamically while under sail
 * @param controllerState - contains time, integral, previous error, etc
 * @param windState - current wind speed and direction
 * @param sailingState - current velocity, heading, heel angle, etc
 * @param activeCoeffs - the currently active PID coefficients based on sailing state
 * @param tackingState - state variables for tacking maneuvers
 * @param gybingState - state variables for gybing maneuvers
 */
typedef struct {
	ControllerState controllerState;
	WindState windState;
	SailingState sailingState;
    PIDcoefficients activeCoeffs;
    TackingState tackingState;
    GybingState gybingState;
} PIDControllerLive;

/* This struct contains the dynamic state of the controller
 * @param integralError - accumulated integral error (degrees * seconds)
 * @param previousError - last error value (degrees)
 * @param filteredError - low-pass filtered error for derivative calculation (degrees)
 * @param previousFilteredError - last filtered error (degrees)
 * @param lastTime - timestamp of last update (milliseconds)
 * @param currentTime - current timestamp (milliseconds)
 */
typedef struct {
	float integralError;
	float previousError;
	float filteredError;
	float previousFilteredError;
	uint32_t lastTime;
	uint32_t currentTime;
} ControllerState;

/* This struct contains the current wind state
 * @param windSpeed - current wind speed (m/s)
 * @param windDirection - current wind direction (degrees from North CCW)
 */
typedef struct {
	float windSpeed;
	float windDirection;
} WindState;

/* This struct contains the current sailing state
 * @param linearVelocity - current over-water velocity (m/s)
 * @param angularVelocity - current rotational velocity (rad/s)
 * @param heelAngle - current heel angle (degrees from vertical, windward side positive)
 * @param desiredHeading - desired heading (degrees from North CCW)
 * @param currentHeading - current heading (degrees from North CCW)
 */
typedef struct {
	float linearVelocity;
	float angularVelocity;
	float heelAngle;
	float desiredHeading;
	float currentHeading;
} SailingState;

/* This struct contains state variables for tacking maneuvers
 * @param isTacking - whether the boat is currently tacking
 * @param tackingStartTime - timestamp when tacking started (milliseconds)
 * @param tackingDuration - expected duration of the tack (milliseconds)
 * @param initialHeading - heading at the start of the tack (degrees from North CCW)
 * @param targetHeading - desired heading after the tack (degrees from North CCW)
 */
typedef struct {
    bool isTacking;
    float tackingStartTime;
    float tackingDuration;
    float initialHeading;
    float targetHeading;
} TackingState;

/* This struct contains state variables for gybing maneuvers
 * @param isGybing - whether the boat is currently gybing
 * @param gybingStartTime - timestamp when gybing started (milliseconds)
 * @param gybingDuration - expected duration of the gybe (milliseconds)
 * @param initialHeading - heading at the start of the gybe (degrees from North CCW)
 * @param targetHeading - desired heading after the gybe (degrees from North CCW)
 */
typedef struct {
    bool isGybing;
    float gybingStartTime;
    float gybingDuration;
    float initialHeading;
    float targetHeading;
} GybingState;

/* This struct encapsulates the entire PID controller
 * @param live - the live state of the controller
 * @param fixed - the fixed parameters of the controller
 */
typedef struct {
    PIDControllerLive live;
    PIDControllerFixed fixed;
} PIDController;


PIDController initController(PIDControllerFixed fixed, PIDControllerLive live) {
    PIDController controller;
    controller.fixed = fixed;
    controller.live = live;
    return controller;
}

PIDControllerLive initLiveController() {
    PIDControllerLive live;
    // initialize controller state - time, integral, etc
    ControllerState cState;
    cState.integralError = 0;
    cState.previousError = 0;
    cState.filteredError = 0;
    cState.previousFilteredError = 0;
    cState.lastTime  = HAL_GetTick();
    cState.currentTime = cState.lastTime;
    // get wind data and initialize wind state
    WindState wState;
    wState.windSpeed = getWindSpeed();
    wState.windDirection = getWindDirection();
    // initialize sailing state and get IMU data
    SailingState sState;
    sState.linearVelocity = getLinearVelocity();
    sState.angularVelocity = getAngularVelocity();
    sState.heelAngle = getHeelAngle();
    sState.currentHeading = getCurrentHeading();
    sState.desiredHeading = getDesiredHeading();
    // assign states to live controller
    PIDcoefficients activeCoeffs;

    live.activeCoeffs = activeCoeffs;
    live.controllerState = cState;
    live.windState = wState;
    live.sailingState = sState;
    return live;
}

// Initializes PID controller parameters
PIDController initPID(PIDControllerFixed fixedController) {
    PIDControllerLive liveController = initLiveController();
    PIDController controller = initController(fixedController, liveController);
}

void updateControllerVariables (PIDController controller) {
    // Update the sailing and sea state in the controller
    controller.live.windState.windSpeed = getWindSpeed();
    controller.live.windState.windDirection = getWindDirection();
    controller.live.sailingState.linearVelocity = getLinearVelocity();
    controller.live.sailingState.angularVelocity = getAngularVelocity();
    controller.live.sailingState.heelAngle = getHeelAngle();
    controller.live.sailingState.currentHeading = getCurrentHeading();
    controller.live.sailingState.desiredHeading = getDesiredHeading();
}

// Gets a new rudder angle based on the current error
float getRudderAngle(PIDController controller, float currentError) {
    PIDcoefficients PID = controller.live.activeCoeffs;
    ControllerState cState = controller.live.controllerState;
    ScalingCoefficients scaling = controller.fixed.scalingCoeffs;
    PhysicalParams params = controller.fixed.physicalParams;

	cState.currentTime = HAL_GetTick();
	float dt = cState.currentTime -  cState.lastTime;
    // Calculating the integral addition
    float integral = cState.integralError + currentError * dt;
    // Low pass filtering the derivative
    cState.filteredError = PID.derivativeFilterFactor * currentError + (1 - PID.derivativeFilterFactor) * cState.previousFilteredError;
    // Calculating the derivative
    float derivative = (cState.filteredError - cState.previousFilteredError) / dt;
    // Calculating PID output
    float outputAngle = PID.Kp * currentError + PID.Ki * integral + PID.Kd * derivative;
    // Saving error
    cState.previousFilteredError = cState.filteredError;
    // Clamping output and integral to prevent windup
    if (outputAngle > params.outputMax) {
        outputAngle = params.outputMax;
        if(currentError < 0) {
            cState.integralError += currentError * dt; 
        }
    } else if (outputAngle < params.outputMin) {
        outputAngle = params.outputMin;
        if(currentError > 0) {
            cState.integralError += currentError * dt; 
        }
    } else {
        cState.integralError += currentError * dt;
    }
    // Apply integral decay
    cState.integralError *= PID.integralDecayFactor;
    cState.previousError = currentError;
    cState.lastTime = cState.currentTime;
    return outputAngle;
}

// Updates the controller state without generating new output
void updateController(PIDController controller, float currentError) {
    PIDcoefficients PID = controller.live.activeCoeffs;
    ControllerState cState = controller.live.controllerState;

	cState.currentTime = HAL_GetTick();
    cState.previousError = currentError;
    cState.lastTime = cState.currentTime;
    cState.integralError = cState.integralError * PID.integralDecayFactor;
}

// Runs control model based on current sailing state
void runPID(PIDController controller, float *rudderAngle) {
    WindState wind = controller.live.windState;
    SailingState sailing = controller.live.sailingState;

    float error = sailing.currentHeading - sailing.desiredHeading;
    // Normalize error to be within -180 to 180 degrees
    if(error > 180){
        error -= 360;
    }
    else if(error < -180){
        error += 360;
    }
    
    State state = getState(error, controller);

	switch(state){
	case STRAIGHT:
		*rudderAngle = straightLine(controller, error);
        break;
	case TACKING:
		*rudderAngle = tacking(controller, error);
		break;
	case GYBING:
		*rudderAngle = gybing(controller, error);
		break;
	case LOWWIND:
		*rudderAngle = lowwind(controller, error);
		break;
	case IRONS:
		*rudderAngle = irons(controller, error);
		break;
	}
    
}

State getState(float error, PIDController controller) {
    PIDcoefficients PID = controller.live.activeCoeffs;
    PhysicalParams params = controller.fixed.physicalParams;    
    SailingState sailing = controller.live.sailingState;
    WindState wind = controller.live.windState;
    
    if (isTackingCondition(controller, error)) {
        return TACKING;
    } else if (isGybingCondition(controller, error)) {
        return GYBING;
    } else if (wind.windSpeed < params.lowWindThreshold) {
        return LOWWIND;
    } else if (abs(wind.windDirection - sailing.currentHeading) < params.upwindIronsAngle || abs(wind.windDirection - sailing.currentHeading) < params.downwindIronsAngle) {
        return IRONS;
    } else {
        return STRAIGHT; // Default to straight if no other conditions met
    }
}

//To do, check if these conditions are correct - sign conventions, etc.
bool isTackingCondition(PIDController controller, float error) {
    WindState wind = controller.live.windState;
    SailingState sailing = controller.live.sailingState;

    float relativeWind = wind.windDirection - sailing.desiredHeading;
    float boatWindAngle = wind.windDirection - sailing.currentHeading;

    if(controller.live.tackingState.isTacking) {
        // If already tacking, continue until duration is over
        if(HAL_GetTick() - controller.live.tackingState.tackingStartTime < controller.live.tackingState.tackingDuration) {
            return true;
        } else {
            controller.live.tackingState.isTacking = false;
            return false;
        }
    } else if(abs(boatWindAngle) < 90 || abs(boatWindAngle) > 270) {
        // Check if the desired heading is on the opposite side of the wind direction
        if((relativeWind > 0 && boatWindAngle < 0) || (relativeWind < 0 && boatWindAngle > 0)){
            return true;
        }
    }
    return false;    
}

//To do, check if these conditions are correct - sign conventions, etc.
bool isGybingCondition(PIDController controller, float error) {
    WindState wind = controller.live.windState;
    SailingState sailing = controller.live.sailingState;

    float relativeWind = wind.windDirection - sailing.desiredHeading;
    float boatWindAngle = wind.windDirection - sailing.currentHeading;

    if(abs(boatWindAngle) > 90 && abs(boatWindAngle) < 270) {
        if((relativeWind > 0 && boatWindAngle < 0) || (relativeWind < 0 && boatWindAngle > 0)){
            return true;
        }
    }
    return false;    
}

// Control model for straight line sailing
float straightLine(PIDController controller, float error) {
    PIDcoefficients PID = controller.live.activeCoeffs;
    ScalingCoefficients scaling = controller.fixed.scalingCoeffs;
    PhysicalParams params = controller.fixed.physicalParams;
    SailingState sailing = controller.live.sailingState;
    WindState wind = controller.live.windState;

    controller.live.activeCoeffs = controller.fixed.standardCoeffs;

    float rudderAngle;
	
    // if within error threshold, do not adjust rudder angle
	if(abs(error) < PID.errorThreshold){
        // update controller state without changing output
		updateControllerTime(controller, error);
        rudderAngle = 0;
	}
	else {
        // compute scaled angle from PID output
        rudderAngle = (scaling.velocityFactor/pow(sailing.linearVelocity, 2.0))*(1-scaling.heelFactor*sailing.heelAngle)*getRudderAngle(controller, error);
        // limit to max rudder angle
        if(rudderAngle > params.outputMax) {
            rudderAngle = params.outputMax;
        }
        else if(rudderAngle < params.outputMin) {
            rudderAngle = params.outputMin;
        }
	}
    return rudderAngle;
}

float tacking(PIDController controller, float error) {
    PIDcoefficients PID = controller.live.activeCoeffs;
    SailingState sailing = controller.live.sailingState;
    WindState wind = controller.live.windState;
    TackingState tacking = controller.live.tackingState;
    PhysicalParams params = controller.fixed.physicalParams;
    ScalingCoefficients scaling = controller.fixed.scalingCoeffs;

    tacking.isTacking = true;
    tacking.tackingStartTime = HAL_GetTick();

    tacking.tackingDuration = 5000 * scaling.tackTimeFactor; // Placeholder duration, should be tuned
    tacking.initialHeading = sailing.currentHeading;
    if(tacking.initialHeading - sailing.desiredHeading > 0) {
        // Tack to starboard
        tacking.targetHeading = sailing.desiredHeading + scaling.tackHeadingPadding;
    } else {
        // Tack to port
        tacking.targetHeading = sailing.desiredHeading - scaling.tackHeadingPadding;
    }   
    // Normalize target heading to be within 0-360 degrees
    if(tacking.targetHeading > 360) {
        tacking.targetHeading -= 360;
    } else if(tacking.targetHeading < 0) {
        tacking.targetHeading += 360;
    }

    controller.live.activeCoeffs = controller.fixed.tackingCoeffs;
}

float gybing(PIDController controller, float error) {

    controller.live.activeCoeffs = controller.fixed.gybingCoeffs;
}

float lowwind(PIDController controller, float error) {
}

float irons(PIDController controller, float error) {
    PhysicalParams params = controller.fixed.physicalParams;
    SailingState sailing = controller.live.sailingState;
    WindState wind = controller.live.windState;

    if (abs(wind.windDirection - sailing.currentHeading) < params.upwindIronsAngle) {
        // If upwind irons, turn to starboard
        // arbitrary angle to turn
    } else if (abs(wind.windDirection - sailing.currentHeading) < params.downwindIronsAngle) {
        // If downwind irons, turn to port
        getRudderAngle(controller, 30); // arbitrary angle to turn
    }
}







