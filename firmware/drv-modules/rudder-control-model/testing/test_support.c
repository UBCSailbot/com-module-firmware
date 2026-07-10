#include <stdint.h>
#include <string.h>

#include "../RUDDER.h"

uint32_t fake_tick_ms;

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms;
}

static PIDControllerFixed test_params(void)
{
    PIDControllerFixed params;
    memset(&params, 0, sizeof(params));

    params.standardCoeffs.Kp = 2.0f;
    params.standardCoeffs.Ki = 0.5f;
    params.standardCoeffs.Kd = 0.25f;
    params.standardCoeffs.integralDecayFactor = 1.0f;
    params.standardCoeffs.derivativeFilterFactor = 1.0f;
    params.standardCoeffs.errorThreshold = 1.0f;
    params.standardCoeffs.headingTolerance = 5.0f;
    params.standardCoeffs.angVelTolerance = 0.2f;
    params.standardCoeffs.integralMax = 100.0f;

    params.tackingCoeffs = params.standardCoeffs;
    params.tackingCoeffs.headingTolerance = 5.0f;
    params.gybingCoeffs = params.standardCoeffs;
    params.gybingCoeffs.headingTolerance = 5.0f;
    params.lowWindCoeffs = params.standardCoeffs;

    params.scalingCoeffs.tackTime = 2.0f;
    params.scalingCoeffs.gybeTime = 2.0f;
    params.scalingCoeffs.tackHeadingPadding = 10.0f;
    params.scalingCoeffs.gybeHeadingPadding = 10.0f;
    params.scalingCoeffs.velocityFactor = 1.0f;
    params.scalingCoeffs.heelFactor = 0.0f;
    params.scalingCoeffs.averageWindowSize = 10;

    params.physicalParams.outputMin = -30.0f;
    params.physicalParams.outputMax = 30.0f;
    params.physicalParams.lowWindThreshold = 1.0f;
    params.physicalParams.upwindIronsAngle = 45.0f;
    params.physicalParams.downwindIronsAngle = 45.0f;

    params.stateThresholds.ironsSpeed = 0.2f;
    params.stateThresholds.stateironsRot = 0.1f;
    params.stateThresholds.tackingRotThreshold = 0.5f;
    params.stateThresholds.gybingRotThreshold = 0.5f;

    return params;
}

void reset_test_controller(void)
{
    fake_tick_ms = 1000;
    initController(test_params());
    controller.live.sailingState.linearVelocity = 2.0f;
    controller.live.sailingState.angularVelocity = 0.5f;
    controller.live.windState.windSpeed = 5.0f;
    controller.live.tackingState.tackingAllowed = true;
    controller.live.gybingState.gybingAllowed = true;
}
