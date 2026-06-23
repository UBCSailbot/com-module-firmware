/*ESTIMATOR.H*/

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "RUDDER.h"

typedef struct {
    float heading;
    float avgHeading;
    float yawRate;
    float avgYawRate;

    float relWindAngle;
    float avgRelWindAngle;

    float windSpeed;
    float avgWindSpeed;

    float linearVel;
    float heelAngle;

    uint32_t lastUpdateMs;
    float prevHeading;

    bool initialized;
    bool fault_detected;
} StateEstimate;

void updateStateEstimate(
    StateEstimate *est,
    float measuredHeadingDeg,
    float measuredRelWindAngleDeg,
    float measuredWindSpeed,
    float measuredLinearVelocity,
    float measuredHeelAngleDeg
);

void applyEstimateToController(PIDController *controller, StateEstimate *est);
void Estimator_UpdateController(PIDController *controller);

#endif /* ESTIMATOR_H_ */
