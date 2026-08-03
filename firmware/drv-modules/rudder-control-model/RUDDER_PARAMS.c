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

typedef enum {
    RUDDER_PARAM_FLOAT,
    RUDDER_PARAM_INT
} RudderParamType;

typedef struct {
    RudderParamId id;
    void *value;
    RudderParamType type;
} RudderParamEntry;

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

static RudderParamEntry rudderParamTable[] = {
    { STANDARD_KP, &rudderFixedParams.standardCoeffs.Kp, RUDDER_PARAM_FLOAT },
    { STANDARD_KI, &rudderFixedParams.standardCoeffs.Ki, RUDDER_PARAM_FLOAT },
    { STANDARD_KD, &rudderFixedParams.standardCoeffs.Kd, RUDDER_PARAM_FLOAT },
    { STANDARD_INTEGRAL_MAX, &rudderFixedParams.standardCoeffs.integralMax, RUDDER_PARAM_FLOAT },
    { STANDARD_INTEGRAL_DECAY, &rudderFixedParams.standardCoeffs.integralDecayFactor, RUDDER_PARAM_FLOAT },
    { STANDARD_DERIVATIVE_FILTER, &rudderFixedParams.standardCoeffs.derivativeFilterFactor, RUDDER_PARAM_FLOAT },
    { STANDARD_ERROR_THRESHOLD, &rudderFixedParams.standardCoeffs.errorThreshold, RUDDER_PARAM_FLOAT },
    { STANDARD_HEADING_TOLERANCE, &rudderFixedParams.standardCoeffs.headingTolerance, RUDDER_PARAM_FLOAT },
    { STANDARD_ANG_VEL_TOLERANCE, &rudderFixedParams.standardCoeffs.angVelTolerance, RUDDER_PARAM_FLOAT },

    { TACKING_KP, &rudderFixedParams.tackingCoeffs.Kp, RUDDER_PARAM_FLOAT },
    { TACKING_KI, &rudderFixedParams.tackingCoeffs.Ki, RUDDER_PARAM_FLOAT },
    { TACKING_KD, &rudderFixedParams.tackingCoeffs.Kd, RUDDER_PARAM_FLOAT },
    { TACKING_INTEGRAL_MAX, &rudderFixedParams.tackingCoeffs.integralMax, RUDDER_PARAM_FLOAT },
    { TACKING_INTEGRAL_DECAY, &rudderFixedParams.tackingCoeffs.integralDecayFactor, RUDDER_PARAM_FLOAT },
    { TACKING_DERIVATIVE_FILTER, &rudderFixedParams.tackingCoeffs.derivativeFilterFactor, RUDDER_PARAM_FLOAT },
    { TACKING_ERROR_THRESHOLD, &rudderFixedParams.tackingCoeffs.errorThreshold, RUDDER_PARAM_FLOAT },
    { TACKING_HEADING_TOLERANCE, &rudderFixedParams.tackingCoeffs.headingTolerance, RUDDER_PARAM_FLOAT },
    { TACKING_ANG_VEL_TOLERANCE, &rudderFixedParams.tackingCoeffs.angVelTolerance, RUDDER_PARAM_FLOAT },

    { GYBING_KP, &rudderFixedParams.gybingCoeffs.Kp, RUDDER_PARAM_FLOAT },
    { GYBING_KI, &rudderFixedParams.gybingCoeffs.Ki, RUDDER_PARAM_FLOAT },
    { GYBING_KD, &rudderFixedParams.gybingCoeffs.Kd, RUDDER_PARAM_FLOAT },
    { GYBING_INTEGRAL_MAX, &rudderFixedParams.gybingCoeffs.integralMax, RUDDER_PARAM_FLOAT },
    { GYBING_INTEGRAL_DECAY, &rudderFixedParams.gybingCoeffs.integralDecayFactor, RUDDER_PARAM_FLOAT },
    { GYBING_DERIVATIVE_FILTER, &rudderFixedParams.gybingCoeffs.derivativeFilterFactor, RUDDER_PARAM_FLOAT },
    { GYBING_ERROR_THRESHOLD, &rudderFixedParams.gybingCoeffs.errorThreshold, RUDDER_PARAM_FLOAT },
    { GYBING_HEADING_TOLERANCE, &rudderFixedParams.gybingCoeffs.headingTolerance, RUDDER_PARAM_FLOAT },
    { GYBING_ANG_VEL_TOLERANCE, &rudderFixedParams.gybingCoeffs.angVelTolerance, RUDDER_PARAM_FLOAT },

    { LOW_WIND_KP, &rudderFixedParams.lowWindCoeffs.Kp, RUDDER_PARAM_FLOAT },
    { LOW_WIND_KI, &rudderFixedParams.lowWindCoeffs.Ki, RUDDER_PARAM_FLOAT },
    { LOW_WIND_KD, &rudderFixedParams.lowWindCoeffs.Kd, RUDDER_PARAM_FLOAT },
    { LOW_WIND_INTEGRAL_MAX, &rudderFixedParams.lowWindCoeffs.integralMax, RUDDER_PARAM_FLOAT },
    { LOW_WIND_INTEGRAL_DECAY, &rudderFixedParams.lowWindCoeffs.integralDecayFactor, RUDDER_PARAM_FLOAT },
    { LOW_WIND_DERIVATIVE_FILTER, &rudderFixedParams.lowWindCoeffs.derivativeFilterFactor, RUDDER_PARAM_FLOAT },
    { LOW_WIND_ERROR_THRESHOLD, &rudderFixedParams.lowWindCoeffs.errorThreshold, RUDDER_PARAM_FLOAT },
    { LOW_WIND_HEADING_TOLERANCE, &rudderFixedParams.lowWindCoeffs.headingTolerance, RUDDER_PARAM_FLOAT },
    { LOW_WIND_ANG_VEL_TOLERANCE, &rudderFixedParams.lowWindCoeffs.angVelTolerance, RUDDER_PARAM_FLOAT },

    { VELOCITY_FACTOR, &rudderFixedParams.scalingCoeffs.velocityFactor, RUDDER_PARAM_FLOAT },
    { HEEL_FACTOR, &rudderFixedParams.scalingCoeffs.heelFactor, RUDDER_PARAM_FLOAT },
    { TACK_TIME, &rudderFixedParams.scalingCoeffs.tackTime, RUDDER_PARAM_FLOAT },
    { GYBE_TIME, &rudderFixedParams.scalingCoeffs.gybeTime, RUDDER_PARAM_FLOAT },
    { TACK_HEADING_PADDING, &rudderFixedParams.scalingCoeffs.tackHeadingPadding, RUDDER_PARAM_FLOAT },
    { GYBE_HEADING_PADDING, &rudderFixedParams.scalingCoeffs.gybeHeadingPadding, RUDDER_PARAM_FLOAT },
    { AVERAGE_WINDOW_SIZE, &rudderFixedParams.scalingCoeffs.averageWindowSize, RUDDER_PARAM_INT },

    { OUTPUT_MAX, &rudderFixedParams.physicalParams.outputMax, RUDDER_PARAM_FLOAT },
    { OUTPUT_MIN, &rudderFixedParams.physicalParams.outputMin, RUDDER_PARAM_FLOAT },
    { UPWIND_IRONS_ANGLE, &rudderFixedParams.physicalParams.upwindIronsAngle, RUDDER_PARAM_FLOAT },
    { DOWNWIND_IRONS_ANGLE, &rudderFixedParams.physicalParams.downwindIronsAngle, RUDDER_PARAM_FLOAT },
    { LOW_WIND_THRESHOLD, &rudderFixedParams.physicalParams.lowWindThreshold, RUDDER_PARAM_FLOAT },

    { STATE_LOW_WIND_THRESHOLD, &rudderFixedParams.stateThresholds.lowWindThreshold, RUDDER_PARAM_FLOAT },
    { TACKING_LIN_THRESHOLD, &rudderFixedParams.stateThresholds.tackingLinThreshold, RUDDER_PARAM_FLOAT },
    { TACKING_ROT_THRESHOLD, &rudderFixedParams.stateThresholds.tackingRotThreshold, RUDDER_PARAM_FLOAT },
    { GYBING_LIN_THRESHOLD, &rudderFixedParams.stateThresholds.gybingLinThreshold, RUDDER_PARAM_FLOAT },
    { GYBING_ROT_THRESHOLD, &rudderFixedParams.stateThresholds.gybingRotThreshold, RUDDER_PARAM_FLOAT },
    { IRONS_SPEED, &rudderFixedParams.stateThresholds.ironsSpeed, RUDDER_PARAM_FLOAT },
    { STATE_IRONS_ROT, &rudderFixedParams.stateThresholds.stateironsRot, RUDDER_PARAM_FLOAT },
};

static uint32_t getRudderParamTableSize() {
    return sizeof(rudderParamTable) / sizeof(rudderParamTable[0]);
}

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

bool setRudderParam(RudderParamId id, float value) {
    for (uint32_t i = 0; i < getRudderParamTableSize(); i++) {
        RudderParamEntry *entry = &rudderParamTable[i];

        if (entry->id == id) {
            if (entry->type == RUDDER_PARAM_INT) {
                int intValue = (int)value;
                if (value != (float)intValue) {
                    return false;
                }
                *((int *)entry->value) = intValue;
            } else {
                *((float *)entry->value) = value;
            }

            return true;
        }
    }

    return false;
}

bool getRudderParam(RudderParamId id, float *value) {
    if (value == NULL) {
        return false;
    }

    for (uint32_t i = 0; i < getRudderParamTableSize(); i++) {
        RudderParamEntry *entry = &rudderParamTable[i];

        if (entry->id == id) {
            if (entry->type == RUDDER_PARAM_INT) {
                *value = (float)(*((int *)entry->value));
            } else {
                *value = *((float *)entry->value);
            }

            return true;
        }
    }

    return false;
}
