#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
#include "STATE_MACHINE.h"

static uint32_t fake_tick_ms;

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms;
}

float getRudderAngle(float currentError);

#define ASSERT_TRUE(expr)                                                        \
    do {                                                                         \
        if (!(expr)) {                                                           \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);              \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define ASSERT_EQ_INT(expected, actual)                                          \
    do {                                                                         \
        int expected_value = (int)(expected);                                    \
        int actual_value = (int)(actual);                                        \
        if (expected_value != actual_value) {                                    \
            printf(                                                              \
                "FAIL %s:%d: expected %d, got %d\n",                            \
                __FILE__,                                                        \
                __LINE__,                                                        \
                expected_value,                                                  \
                actual_value                                                     \
            );                                                                   \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define ASSERT_NEAR(expected, actual, tolerance)                                 \
    do {                                                                         \
        float expected_value = (float)(expected);                                \
        float actual_value = (float)(actual);                                    \
        float tolerance_value = (float)(tolerance);                              \
        if (fabsf(expected_value - actual_value) > tolerance_value) {            \
            printf(                                                              \
                "FAIL %s:%d: expected %.3f, got %.3f\n",                        \
                __FILE__,                                                        \
                __LINE__,                                                        \
                expected_value,                                                  \
                actual_value                                                     \
            );                                                                   \
            return false;                                                        \
        }                                                                        \
    } while (0)

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
    params.physicalParams.upwindIronsRange = 15.0f;
    params.physicalParams.downwindIronsRange = 15.0f;

    params.stateThresholds.ironsSpeed = 0.2f;
    params.stateThresholds.stateironsRot = 0.1f;
    params.stateThresholds.tackingRotThreshold = 0.5f;
    params.stateThresholds.gybingRotThreshold = 0.5f;

    return params;
}

static void reset_test_controller(void)
{
    fake_tick_ms = 1000;
    initController(test_params());
    controller.live.sailingState.linearVelocity = 2.0f;
    controller.live.sailingState.angularVelocity = 0.5f;
    controller.live.windState.windSpeed = 5.0f;
}

static bool test_state_machine_defaults_to_straight(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 100.0f;
    controller.live.sailingState.desiredHeading = 110.0f;

    RudderSM_Update(&controller, -10.0f);

    ASSERT_EQ_INT(STRAIGHT, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_enters_low_wind(void)
{
    reset_test_controller();

    controller.live.windState.windSpeed = 0.5f;
    controller.live.windState.windDirection = 90.0f;
    controller.live.sailingState.currentHeading = 100.0f;
    controller.live.sailingState.desiredHeading = 110.0f;

    RudderSM_Update(&controller, -10.0f);

    ASSERT_EQ_INT(LOWWIND, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_enters_irons(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 5.0f;
    controller.live.sailingState.desiredHeading = 90.0f;
    controller.live.sailingState.linearVelocity = 0.1f;
    controller.live.sailingState.angularVelocity = 0.05f;

    RudderSM_Update(&controller, -85.0f);

    ASSERT_EQ_INT(IRONS, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_enters_tacking(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 30.0f;
    controller.live.sailingState.desiredHeading = 330.0f;

    RudderSM_Update(&controller, 60.0f);

    ASSERT_EQ_INT(TACKING, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_enters_gybing(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 150.0f;
    controller.live.sailingState.desiredHeading = 210.0f;

    RudderSM_Update(&controller, -60.0f);

    ASSERT_EQ_INT(GYBING, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_rejects_blocked_transition(void)
{
    reset_test_controller();

    controller.live.windState.windSpeed = 0.5f;
    RudderSM_BlockTransition(
        &controller.live.stateMachine.transitionGuards,
        STRAIGHT,
        LOWWIND,
        1000
    );

    RudderSM_Update(&controller, 0.0f);
    ASSERT_EQ_INT(STRAIGHT, controller.live.stateMachine.currentState);

    fake_tick_ms += 1001;
    RudderSM_Update(&controller, 0.0f);
    ASSERT_EQ_INT(LOWWIND, controller.live.stateMachine.currentState);
    return true;
}

static bool test_pid_proportional_output(void)
{
    reset_test_controller();

    controller.fixed.standardCoeffs.Ki = 0.0f;
    controller.fixed.standardCoeffs.Kd = 0.0f;
    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
    controller.live.controllerState.lastTime = fake_tick_ms;

    fake_tick_ms += 100;
    float output = getRudderAngle(10.0f);

    ASSERT_NEAR(20.0f, output, 0.001f);
    ASSERT_NEAR(10.0f, controller.live.liveValues.errorValue, 0.001f);
    return true;
}

static bool test_pid_clamps_output_high(void)
{
    reset_test_controller();

    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
    controller.live.controllerState.lastTime = fake_tick_ms;

    fake_tick_ms += 100;
    float output = getRudderAngle(100.0f);

    ASSERT_NEAR(30.0f, output, 0.001f);
    return true;
}

static bool test_pid_clamps_output_low(void)
{
    reset_test_controller();

    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
    controller.live.controllerState.lastTime = fake_tick_ms;

    fake_tick_ms += 100;
    float output = getRudderAngle(-100.0f);

    ASSERT_NEAR(-30.0f, output, 0.001f);
    return true;
}

static bool test_pid_derivative_uses_elapsed_time(void)
{
    reset_test_controller();

    controller.live.activeCoeffs = controller.fixed.standardCoeffs;
    controller.live.controllerState.lastTime = fake_tick_ms;
    controller.live.controllerState.previousFilteredError = 2.0f;

    fake_tick_ms += 200;
    (void)getRudderAngle(4.0f);

    ASSERT_NEAR(10.0f, controller.live.liveValues.derivativeValue, 0.001f);
    return true;
}

typedef bool (*test_fn)(void);

typedef struct {
    const char *name;
    test_fn run;
} TestCase;

int main(void)
{
    const TestCase tests[] = {
        {"state machine defaults to straight", test_state_machine_defaults_to_straight},
        {"state machine enters low wind", test_state_machine_enters_low_wind},
        {"state machine enters irons", test_state_machine_enters_irons},
        {"state machine enters tacking", test_state_machine_enters_tacking},
        {"state machine enters gybing", test_state_machine_enters_gybing},
        {"state machine rejects blocked transition", test_state_machine_rejects_blocked_transition},
        {"pid proportional output", test_pid_proportional_output},
        {"pid clamps output high", test_pid_clamps_output_high},
        {"pid clamps output low", test_pid_clamps_output_low},
        {"pid derivative uses elapsed time", test_pid_derivative_uses_elapsed_time},
    };
    int failures = 0;

    for (unsigned int i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        if (tests[i].run()) {
            printf("PASS %s\n", tests[i].name);
        } else {
            printf("FAIL %s\n", tests[i].name);
            failures++;
        }
    }

    if (failures > 0) {
        printf("%d test(s) failed\n", failures);
        return 1;
    }

    printf("All rudder-control-model tests passed\n");
    return 0;
}
