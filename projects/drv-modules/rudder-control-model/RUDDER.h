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

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

extern PIDController PIDController;

/* This struct represents the overall PID controller fixed coefficients
 * These coeffs are fixed as they do not change while the boat is under sail
 * @param standardCoeffs - a series of PID coefficients and related factors that correspond to straight line sailing
 * @param tackingCoeffs - PID coeffs and the like for tacking
 * @param gybingCoeffs - PID coeffs and the like for gybing
 * @param lowWindCoeffs - PID coeffs etc for low wind sailing
 * @param scalingCoeffs - scale factors for velocity, roll, etc
 * @param stateThresholds - thresholding values for determining sailing state
 */
typedef struct PIDControllerFixed_tag {
	PIDcoefficients standardCoeffs;
	PIDcoefficients tackingCoeffs;
	PIDcoefficients gybingCoeffs;
	PIDcoefficients lowWindCoeffs;
	ScalingCoefficients scalingCoeffs;
	PhysicalParams physicalParams;
	StateThresholds stateThresholds;
} PIDControllerFixed;

/* This struct gives coefficients for a PID controller
 * @param Kp - proportional gain (unitless)
 * @param Kd - derivative gain (seconds)
 * @param Ki - integral gain (1/seconds)
 * @param integralMax - maximum integral value (point at which controller saturates) (degrees)
 * @param integralDecayFactor - corresponds to how "leaky" our integral is (0-1, unitless)
 * @param derivativeFilterFactor - determine show sensitive derivative term is (0-1, unitless)
 * @param errorThreshold - when PID controller sets new angle (degrees)
 */
typedef struct PIDcoefficients_tag {
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
typedef struct ScalingCoefficients_tag {
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
typedef struct PhysicalParams_tag {
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
typedef struct StateThresholds_tag {
	float lowWindThreshold;
	float tackingLinThreshold;
	float tackingRotThreshold;
	float gybingLinThreshold;
	float gybingRotThreshold;
	float ironsSpeed;
	float stateironsRot;
} StateThresholds;

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