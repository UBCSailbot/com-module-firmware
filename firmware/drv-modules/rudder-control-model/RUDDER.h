/* RUDDER.h
 *	Created on: Mar 26, 2025
 *      Author: Emma Duong

 * This library implements a PID model with various extra functionality to account for dynamic sailing conditions
 * It currently has the following functions: 
 * 	- initController - initializes the PID controller with fixed and live parameters
 * 	- runPID - runs the PID controller and generates rudder angles to sail according to the desired heading
 * 	- updateControllerVariables - updates the live controller variables with current sensor readings
 * 	- resetController - resets live controller state variables
 */

#ifndef RUDDER_H_
#define RUDDER_H_

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "MOCK_HARDWARE_FUNCTIONS.h"

#define STRAIGHT_ONLY
#define TUNING_MODE

/* This struct gives coefficients for a PID controller
 * @param Kp - proportional gain (unitless)
 * @param Kd - derivative gain (seconds)
 * @param Ki - integral gain (1/seconds)
 * @param integralMax - maximum integral value (point at which controller saturates) (degrees)
 * @param integralDecayFactor - corresponds to how "leaky" our integral is (0-1, unitless)
 * @param derivativeFilterFactor - determine show sensitive derivative term is (0-1, unitless)
 * @param errorThreshold - when PID controller sets new angle (degrees)
 */
typedef struct{
	float Kp;
	float Kd;
	float Ki;
	float integralMax;
	float integralDecayFactor;
	float derivativeFilterFactor;
	float errorThreshold;
	float headingTolerance;
	float angVelTolerance;
} PIDcoefficients;

/* This struct contains scaling factors for rudder angle
 * @param velocityFactor - scaling factor based on over-water velocity (degrees s^2 / m^2), should be proportional to v^2
 * @param heelFactor - scaling factor based on heel angle (unitless)
 * @param tackTime - tacking max duration (seconds)
 * @param gybeTime - gybing max duration (seconds)
 * @param tackHeadingPadding - additional heading change to add to tacking maneuvers (degrees)
 * @param gybeHeadingPadding - additional heading change to add to gybing maneuvers (degrees)
 * @param averageWindowSize - size of the moving average window for velocity and heading (number of samples)
 */
typedef struct {
	float velocityFactor;
	float heelFactor;
	float tackTime;
	float gybeTime;
	float tackHeadingPadding; 
	float gybeHeadingPadding;
	int averageWindowSize;
} ScalingCoefficients;

/* This struct contains physical system parameters
 * @param outputMax - the maximum rudder angle in degrees (from 0-90 degrees)
 * @param outputMin - the minimum rudder angle in degrees (from -90-0 degrees)
 * @param upwindIronsAngle - the angle to the wind that corresponds to irons (0-90 degrees)
 * @param downwindIronsAngle - angle corresponding to downwind irons (90-180 degrees)
 */
typedef struct {
	float outputMax;
	float outputMin;
	float upwindIronsAngle;
	float downwindIronsAngle;
	float lowWindThreshold;
} PhysicalParams;

/* This struct contains thresholds to determine the state of the boat
 * @param lowWindThreshold - maximum wind speed to be in low wind mode (m/s)
 * @param tackingLinThreshold - linear velocity threshold to be considered tacking (m/s)
 * @param tackingRotThreshold - min rotational velo to be considered tacking (rad/s)
 * @param gybingLinThreshold - linear velo threshold to be gybing (m/s)
 * @param gybingRotThreshold - min rotational velo to be gybing (rad/s)
 */
typedef struct {
	float lowWindThreshold;
	float tackingLinThreshold;
	float tackingRotThreshold;
	float gybingLinThreshold;
	float gybingRotThreshold;
	float ironsSpeed;
	float stateironsRot;
} StateThresholds;

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
    bool isActive;
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
    float averageLinVelocity;
    float averageAngVelocity;
    float averageHeading;
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
    float initialHeading;
    float targetHeading;
} GybingState;

/* This struct represents the overall PID controller fixed coefficients
 * These coeffs are fixed as they do not change while the boat is under sail
 * @param standardCoeffs - a series of PID coefficients and related factors that correspond to straight line sailing
 * @param tackingCoeffs - PID coeffs and the like for tacking
 * @param gybingCoeffs - PID coeffs and the like for gybing
 * @param lowWindCoeffs - PID coeffs etc for low wind sailing
 * @param scalingCoeffs - scale factors for velocity, roll, etc
 * @param stateThresholds - thresholding values for determining sailing state
 */
typedef struct {
	PIDcoefficients standardCoeffs;
	PIDcoefficients tackingCoeffs;
	PIDcoefficients gybingCoeffs;
	PIDcoefficients lowWindCoeffs;
	ScalingCoefficients scalingCoeffs;
	PhysicalParams physicalParams;
	StateThresholds stateThresholds;
} PIDControllerFixed;

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
	volatile SailingState sailingState;
    PIDcoefficients activeCoeffs;
    TackingState tackingState;
    GybingState gybingState;
} PIDControllerLive;

/* This struct encapsulates the entire PID controller
 * @param live - the live state of the controller
 * @param fixed - the fixed parameters of the controller
 */
typedef struct {
    PIDControllerLive live;
    PIDControllerFixed fixed;
} PIDController;

extern PIDController controller;

/* Enumerates the boat's possible states
 * States should be self explanatory to those familiar with the model
 * Or sailing in general
 * Or the project - boats, idk*/
typedef enum {
	STRAIGHT,
	TACKING,
	GYBING,
	LOWWIND,
	IRONS
} State;

// Initializes the PID controller with fixed and live parameters from given fixed params
void initController(PIDControllerFixed fixed);

// runs the PID controller and generates rudder angles to sail according to the desired heading
void runPID(float *rudderAngle);

// Updates the live controller variables with current sensor readings
void updateControllerVariables();

// resets live controller state variables
void resetController();

// Determines the current state of the boat based on sailing conditions
State getState(float error);

#endif /* RUDDER_H_ */
