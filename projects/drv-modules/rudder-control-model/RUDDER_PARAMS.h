/* RUDDER_PARAMS.h
 *  Created on: Oct 18, 2025
 *      Author: Emma Duong
 * 
 * This header file declares the fixed parameters for the PID controller used in Polaris' rudder control system.
 * These parameters include PID coefficients for different sailing states, scaling factors,
 * physical constraints, and state thresholds.
 * 
 * Key Functions:
 * - getRudderFixedParams: Returns the fixed PID controller parameters.
 * - updateRudderFixedParams: Updates the PID coefficients for a specified sailing mode.
 * - updateRudderScalingParams: Updates the scaling coefficients.
 * 
*/

#ifndef RUDDER_PARAMS_H_
#define RUDDER_PARAMS_H_
#include "RUDDER.h"
#include "RUDDER_PARAMS.c"

/* Getter for the fixed PID controller parameters */
PIDControllerFixed getRudderFixedParams();

/* Setter for the PID coefficients in the fixed PID controller parameters
    * @param PIDCoeffs - the new PID coefficients to set
    * @param mode - the sailing mode for which to set the coefficients
    */

void updateRudderFixedParams(PIDcoefficients PIDCoeffs, PIDMode mode);

/* Setter for the scaling coefficients in the fixed PID controller parameters
    * @param scalingCoeffs - pointer to the new ScalingCoefficients struct
    */

void updateRudderScalingParams(const ScalingCoefficients *scalingCoeffs);

/* Getter for pid coefficients
    * @param mode - desired set of coeffs
    */
PIDcoefficients getPIDCoeffs(PIDMode mode);

/* Getter for scaling coefficients */
ScalingCoefficients getScalingCoeffs();

/* Enum for different PID modes corresponding to sailing states */
typedef enum {
    STANDARD,
    TACKING,
    GYBING,
    LOW_WIND
} PIDMode;

#endif /* RUDDER_PARAMS_H_ */