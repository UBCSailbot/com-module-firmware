#include "test_support.h"

float getRudderAngle(float currentError);

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

int main(void)
{
    const TestCase tests[] = {
        {"pid proportional output", test_pid_proportional_output},
        {"pid clamps output high", test_pid_clamps_output_high},
        {"pid clamps output low", test_pid_clamps_output_low},
        {"pid derivative uses elapsed time", test_pid_derivative_uses_elapsed_time},
    };

    return run_tests(tests, sizeof(tests) / sizeof(tests[0]));
}
