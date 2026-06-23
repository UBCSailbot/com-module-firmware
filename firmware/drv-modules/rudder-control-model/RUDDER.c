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
 * 
 * TODO:
    *  - Implement blocking in state transitions to prevent rapid switching
    *  - Test state transition logic
    *  - Implement isInIrons() function
    *  - Validate sign conventions
    * - Fine-tune PID coefficients and thresholds based on real-world testing
 */

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
#include "ESTIMATOR.h"
#include "STATE_MACHINE.h"

// Global controller instance
PIDController controller;

/*Private function declarations*/

static float straightLine(float error);
static float tacking(void);
static float gybing(void);
static float lowwind(float error);
static float irons(float error);

// Initializes the live state of the PID controller
PIDControllerLive initLiveController() {
    PIDControllerLive live;
    memset(&live, 0, sizeof(PIDControllerLive)); // Zero out the struct
    // initialize controller state - time, integral, etc
    ControllerState cState = {0};
    cState.lastTime  = HAL_GetTick();
    cState.currentTime = cState.lastTime;
    // get wind data and initialize wind state
    volatile WindState wState = {0};
    // wState.windSpeed = getWindSpeed();
    // wState.windDirection = getWindDirection();
    // initialize sailing state and get IMU data
    volatile SailingState sState = {0};
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
    TackingState tState = {0};
    tState.isTacking = false;
    tState.tackingAllowed = true;
    tState.tackingBanStartTime = 0.0f;
    tState.tackingStartTime = 0.0f;
    tState.initialHeading = 0.0f;
    tState.targetHeading = 0.0f;
    live.tackingState = tState;
    // initialize gybing state
    GybingState gState = {0};
    gState.isGybing = false;
    gState.gybingAllowed = true;
    gState.gybingBanStartTime = 0.0f;
    gState.gybingStartTime = 0.0f;
    gState.initialHeading = 0.0f;
    gState.targetHeading = 0.0f;
    live.gybingState = gState;
    // assign states to live controller
    LiveValues liveVals = {0};

    PIDcoefficients activeCoeffs = {0};

    live.activeCoeffs = activeCoeffs;
    live.controllerState = cState;
    live.windState = wState;
    live.sailingState = sState;
    live.tackingState = tState;
    live.gybingState = gState;
    live.liveValues = liveVals;
    RudderSM_Init(&live.stateMachine);

    return live;
}

// Initializes PID controller parameters
void initController(PIDControllerFixed fixedController) {
    PIDControllerLive *live = &controller.live;
    *live = initLiveController();
    controller.fixed = fixedController;
}

// Updates live controller variables with current sensor data
void updateControllerVariables () {
    Estimator_UpdateController(&controller);
}

// Resets live controller state variables
void resetController() {
    memset(&controller.live, 0, sizeof(PIDControllerLive));
    // Reset the controller time
    controller.live.controllerState.lastTime  = HAL_GetTick();
    controller.live.controllerState.currentTime = controller.live.controllerState.lastTime;
    // Reset active coefficients to standard
    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
    RudderSM_Init(&controller.live.stateMachine);
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
    float integral = cState->integralError + currentError * dt / 1000.0;
    if (dt < 1.0f){
    		integral = cState->integralError;
    }
//    printf("Time %f \r\n", dt);
    printf("Integral %f \r\n",integral);
    // Clamp integral to prevent windup
    const float integralMax = PID->integralMax; // from your PhysicalParams or a #define
    // if (integral > integralMax) {
    //     integral = integralMax;
    // } else if (integral < -integralMax) {
    //     integral = -integralMax;
    // }

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
        derivative = (cState->filteredError - cState->previousFilteredError) / dt * 1000;
    }   
    vals->derivativeValue = derivative; 
    printf("Derivative: %f\r\n", derivative);
    // Calculating PID output
    float outputAngle = PID->Kp * currentError + PID->Ki * integral - PID->Kd * derivative;
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
            cState->integralError += currentError * dt / 1000; 
         }

    } else if(outputAngle < params->outputMin) {
        outputAngle = params->outputMin;
        if(currentError > 0) {
            cState->integralError += currentError * dt / 1000; 
        }
    } else {
        cState->integralError += currentError * dt / 1000;
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
    if(HAL_GetTick() - controller.live.tackingState.tackingBanStartTime > 30000) {
        controller.live.tackingState.tackingAllowed = true;
    }

    if(HAL_GetTick() - controller.live.gybingState.gybingBanStartTime > 30000) {
        controller.live.gybingState.gybingAllowed = true;
    }

    float error = sailing->currentHeading - sailing->desiredHeading;
    // Normalize error to be within -180 to 180 degrees
    if(error > 180){
        error -= 360;
    }
    else if(error < -180){
        error += 360;
    }
    
    RudderSM_Update(&controller, error);

	switch(controller.live.stateMachine.currentState){
		case STRAIGHT:
			if(HAL_GetTick() - controller.live.ironsState.ironsEndTime > 30000) {
				controller.live.ironsState.isInIrons = false;
			}
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
        default:
            *rudderAngle = straightLine(error);
            break;
	}    
}

State returnState(){
	State state = controller.live.stateMachine.currentState;
	return state;
}

// Control model for straight line sailing
//TODO: Change so that the model returns to below our heading tolerance versus oscillating around the error threshold
static float straightLine(float error) {
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
static float tacking(void) {
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

static float gybing(void) {
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

static float lowwind(float error) {
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

// Will need to figure out what side of the wind we are on - maybe default to straight line mode for ~30 seconds then allow attempted tack again

static float irons(float error) {
    (void)error;

    if(HAL_GetTick() - controller.live.ironsState.ironsBlockDuration > 30000) {
        controller.live.ironsState.isInIrons = false;
    }
    
    if(!controller.live.ironsState.isInIrons) {
        //set irons state
        controller.live.ironsState.isInIrons = true;
        controller.live.ironsState.ironsStartTime = HAL_GetTick();
        //get and set random angle
        srand(HAL_GetTick()); 
        controller.live.tackingState.tackingAllowed = false;
        controller.live.gybingState.gybingAllowed = false;
        controller.live.tackingState.tackingBanStartTime = HAL_GetTick();
        controller.live.gybingState.gybingBanStartTime = HAL_GetTick();

        float randomAngle;
        randomAngle = (float)((rand() % 9) + 7); // Random angle between 7 and 15 degrees
        if(rand() % 2 == 0) {
            randomAngle = -randomAngle; // Randomly choose left or right
        }
        controller.live.ironsState.randomAngle = randomAngle;
    }

    if(!isInIrons()) {
        //exit irons state
        controller.live.ironsState.isInIrons = false;
        controller.live.ironsState.ironsEndTime = HAL_GetTick();

        float desiredHeadingRelWind;
        float currentHeadingRelWind;

        desiredHeadingRelWind = controller.live.sailingState.desiredHeading - controller.live.windState.windDirection;
        currentHeadingRelWind = controller.live.sailingState.currentHeading - controller.live.windState.windDirection;

        if(desiredHeadingRelWind < 90 || desiredHeadingRelWind > 270) {
            if(currentHeadingRelWind < 180) {
                controller.live.ironsState.fixedHeading = controller.live.windState.windDirection - controller.fixed.physicalParams.upwindIronsAngle - 5.0f;
            } else {
                controller.live.ironsState.fixedHeading = controller.live.windState.windDirection + controller.fixed.physicalParams.upwindIronsAngle + 5.0f; 
            }
            
        } else {
            if(currentHeadingRelWind < 180) {
                // Desired heading is to starboard of wind direction, tack to starboard
                controller.live.sailingState.desiredHeading = controller.live.windState.windDirection + controller.fixed.physicalParams.upwindIronsAngle + 5.0f; // Add small padding
            }
            // Desired heading is to port of wind direction, tack to port
            controller.live.sailingState.desiredHeading = controller.live.windState.windDirection - controller.fixed.physicalParams.upwindIronsAngle - 5.0f; // Add small padding
        }
    }

    return controller.live.ironsState.randomAngle;
}

bool isInIrons() {
    return RudderSM_IsInIrons(&controller);
}

