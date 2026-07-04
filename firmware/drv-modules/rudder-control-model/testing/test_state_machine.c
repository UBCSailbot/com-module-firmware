#include "test_support.h"

void blockTransition(TransitionGuards *guards, State from, State to, uint32_t durationMs);
void getState(float error);

static bool test_state_machine_defaults_to_straight(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 100.0f;
    controller.live.sailingState.desiredHeading = 110.0f;

    getState(-10.0f);

    ASSERT_EQ_INT(STRAIGHT, controller.live.stateMachine.nextState);
    return true;
}

static bool test_state_machine_enters_low_wind(void)
{
    reset_test_controller();

    controller.live.windState.windSpeed = 0.5f;
    controller.live.windState.windDirection = 90.0f;
    controller.live.sailingState.currentHeading = 100.0f;
    controller.live.sailingState.desiredHeading = 110.0f;

    getState(-10.0f);

    ASSERT_EQ_INT(LOWWIND, controller.live.stateMachine.nextState);
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

    getState(-85.0f);

    ASSERT_EQ_INT(IRONS, controller.live.stateMachine.nextState);
    return true;
}

static bool test_state_machine_enters_tacking(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = -30.0f;
    controller.live.sailingState.desiredHeading = 30.0f;

    getState(60.0f);

    ASSERT_EQ_INT(TACKING, controller.live.stateMachine.nextState);
    return true;
}

static bool test_state_machine_enters_gybing(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 150.0f;
    controller.live.sailingState.desiredHeading = -30.0f;

    getState(-60.0f);

    ASSERT_EQ_INT(GYBING, controller.live.stateMachine.nextState);
    return true;
}

static bool test_state_machine_blocks_low_wind_transition(void)
{
    reset_test_controller();

    controller.live.windState.windSpeed = 0.5f;
    blockTransition(
        &controller.live.stateMachine.transitionGuards,
        STRAIGHT,
        LOWWIND,
        1000
    );

    controller.live.stateMachine.nextState = LOWWIND;
    ASSERT_TRUE(fake_tick_ms < controller.live.stateMachine.transitionGuards.timestampBlock[STRAIGHT][LOWWIND]);

    fake_tick_ms += 1001;
    ASSERT_TRUE(fake_tick_ms >= controller.live.stateMachine.transitionGuards.timestampBlock[STRAIGHT][LOWWIND]);
    return true;
}

int main(void)
{
    const TestCase tests[] = {
        {"state machine defaults to straight", test_state_machine_defaults_to_straight},
        {"state machine enters low wind", test_state_machine_enters_low_wind},
        {"state machine enters irons", test_state_machine_enters_irons},
        {"state machine enters tacking", test_state_machine_enters_tacking},
        {"state machine enters gybing", test_state_machine_enters_gybing},
        {"state machine blocks low wind transition", test_state_machine_blocks_low_wind_transition},
    };

    return run_tests(tests, sizeof(tests) / sizeof(tests[0]));
}
