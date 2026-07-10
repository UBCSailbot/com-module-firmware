/* RUDDER_PARAMS.c
 *  Created on: Oct 18, 2025
 *      Author: Emma Duong
 * 
 * This file contains the fixed parameters for the PID controller used in Polaris' rudder control system.
 * These parameters include PID coefficients for different sailing states, scaling factors,
 * physical constraints, and state thresholds.
 * 
 * Key Functions:
 * - getRudderFixedParams: Returns the fixed PID controller parameters.
 * - updateRudderFixedParams: Updates the PID coefficients for a specified sailing mode.
 * - updateRudderScalingParams: Updates the scaling coefficients.
 * 
 * The parameters defined here are used to initialize the PID controller and remain constant during operation.
 * 
 * Note: The values provided are placeholders and should be tuned based on real-world testing and performance.
 */

#include "RUDDER.h"
#include "RUDDER_PARAMS.h"

PIDControllerFixed rudderFixedParams = {
    .standardCoeffs = { .Kp = 1.2, .Ki = 0.05, .Kd = 0.02, .derivativeFilterFactor = 0.8,.integralDecayFactor = 0.98,
        .errorThreshold = 2.0f, .headingTolerance = 5.0f, .angVelTolerance = 0.2f, .integralMax = 10000.0f },
    .tackingCoeffs  = { .Kp = 1.8, .Ki = 0.07, .Kd = 0.03, .derivativeFilterFactor = 0.8, .integralDecayFactor = 0.98,
        .errorThreshold = 3.0f, .headingTolerance = 7.0f, .angVelTolerance = 0.3f },
    .gybingCoeffs   = { .Kp = 1.5, .Ki = 0.06, .Kd = 0.025, .derivativeFilterFactor = 0.8, .integralDecayFactor = 0.98,
        .errorThreshold = 3.0f, .headingTolerance = 7.0f, .angVelTolerance = 0.3f },
    .lowWindCoeffs  = { .Kp = 0.9, .Ki = 0.04, .Kd = 0.01, .derivativeFilterFactor = 0.8, .integralDecayFactor = 0.98,
        .errorThreshold = 1.0f, .headingTolerance = 4.0f, .angVelTolerance = 0.15f },

    .physicalParams = {
        .outputMin = -30.0f,
        .outputMax = 30.0f,
        .lowWindThreshold = 1.5f,
        .upwindIronsRange = 45.0f,
        .downwindIronsRange = 45.0f
    },

    .scalingCoeffs = {
        .velocityFactor = 1.0f,
        .heelFactor = 0.1f,
        .tackTime = 1.0f,
        .gybeTime = 1.0f,
        .averageWindowSize = 10
    },

    .stateThresholds = {
        .ironsSpeed = 0.3f,
        .stateironsRot = 0.1f,
        .tackingRotThreshold = 0.5f,
        .gybingRotThreshold = 0.5f
    }
};

PIDControllerFixed getRudderFixedParams() {
    return rudderFixedParams;
}

void updateRudderFixedParams(PIDcoefficients PIDCoeffs, PIDMode mode) {
    switch (mode)
    {
    case STANDARD_COEFFS:
        rudderFixedParams.standardCoeffs = PIDCoeffs;
        break;
    case TACKING_COEFFS:
        rudderFixedParams.tackingCoeffs = PIDCoeffs;
        break;
    case GYBING_COEFFS:
        rudderFixedParams.gybingCoeffs = PIDCoeffs;
        break;
    case LOW_WIND_COEFFS:
        rudderFixedParams.lowWindCoeffs = PIDCoeffs;    
    default:
        break;
    }
}

PIDcoefficients getPIDCoeffs(PIDMode mode) {
    switch (mode)
    {
    case STANDARD_COEFFS:
        return rudderFixedParams.standardCoeffs;
    case TACKING_COEFFS:
        return rudderFixedParams.tackingCoeffs;
    case GYBING_COEFFS:
        return rudderFixedParams.gybingCoeffs;
    case LOW_WIND_COEFFS:
        return rudderFixedParams.lowWindCoeffs;    
    default:
        return rudderFixedParams.standardCoeffs;
    }
}

void updateRudderScalingParams(ScalingCoefficients scalingCoeffs) {
    rudderFixedParams.scalingCoeffs = scalingCoeffs;
}

ScalingCoefficients getScalingCoeffs() {
    return rudderFixedParams.scalingCoeffs;
}
