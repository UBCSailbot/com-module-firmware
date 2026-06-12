/*ESTIMATOR.c
*/

#include "ESTIMATOR.h"

static float wrap360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float lowPass(float previous, float measurement, float alpha)
{
    return alpha * measurement + (1.0f - alpha) * previous;
}

static float lowPassHeading(float previous, float measurement, float alpha)
{
    return wrap360(previous + alpha * wrap180(measurement - previous));
}

void updateStateEstimate(
    StateEstimate *est,
    float measuredHeadingDeg,
    float measuredRelWindAngleDeg,
    float measuredWindSpeed,
    float measuredLinearVelocity,
    float measuredHeelAngleDeg
) {
    uint32_t now = HAL_GetTick();

    measuredHeadingDeg = wrap360(measuredHeadingDeg);
    measuredRelWindAngleDeg = wrap180(measuredRelWindAngleDeg);
    if (measuredWindSpeed < 0.0f) {
        measuredWindSpeed = 0.0f;
    }

    if (!est->initialized) {
        est->heading = measuredHeadingDeg;
        est->avgHeading = measuredHeadingDeg;
        est->prevHeading = measuredHeadingDeg;

        est->yawRate = 0.0f;
        est->avgYawRate = 0.0f;

        est->relWindAngle = measuredRelWindAngleDeg;
        est->avgRelWindAngle = measuredRelWindAngleDeg;

        est->windSpeed = measuredWindSpeed;
        est->avgWindSpeed = measuredWindSpeed;

        est->linearVel = measuredLinearVelocity;
        est->heelAngle = measuredHeelAngleDeg;

        est->lastUpdateMs = now;
        est->initialized = true;
        return;
    }

    float dt = (now - est->lastUpdateMs) / 1000.0f;

    if (dt <= 0.0f) {
        return;
    }

    float headingDelta = wrap180(measuredHeadingDeg - est->prevHeading);
    float rawYawRate = headingDelta / dt;

    const float headingAlpha = 0.4f;
    const float yawRateAlpha = 0.3f;
    const float windAlpha = 0.2f;
    const float windSpeedAlpha = 0.2f;
    const float velocityAlpha = 0.2f;
    const float heelAlpha = 0.2f;

    est->heading = lowPassHeading(est->heading, measuredHeadingDeg, headingAlpha);

    est->avgHeading =
        lowPassHeading(est->avgHeading, est->heading, 0.1f);

    est->yawRate =
        lowPass(est->yawRate, rawYawRate, yawRateAlpha);

    est->avgYawRate =
        lowPass(est->avgYawRate, est->yawRate, 0.1f);

    est->relWindAngle =
        lowPass(est->relWindAngle, measuredRelWindAngleDeg, windAlpha);

    est->avgRelWindAngle =
        lowPass(est->avgRelWindAngle, est->relWindAngle, 0.1f);

    est->windSpeed =
        lowPass(est->windSpeed, measuredWindSpeed, windSpeedAlpha);

    est->avgWindSpeed =
        lowPass(est->avgWindSpeed, est->windSpeed, 0.1f);

    est->linearVel =
        lowPass(est->linearVel, measuredLinearVelocity, velocityAlpha);

    est->heelAngle =
        lowPass(est->heelAngle, measuredHeelAngleDeg, heelAlpha);

    est->prevHeading = measuredHeadingDeg;
    est->lastUpdateMs = now;
}

void applyEstimateToController(PIDController *controller, StateEstimate *est)
{
    controller->live.sailingState.currentHeading = est->heading;
    controller->live.sailingState.averageHeading = est->avgHeading;
    controller->live.sailingState.averageAngVelocity = est->avgYawRate;

    controller->live.windState.relativeWindAngle = est->relWindAngle;
    controller->live.windState.averageRelativeWindAngle = est->avgRelWindAngle;
    controller->live.windState.windSpeed = est->windSpeed;

    controller->live.sailingState.linearVelocity = est->linearVel;
    controller->live.sailingState.heelAngle = est->heelAngle;
}
