#include "test_support.h"
#include "../STATE_MACHINE.h"

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
    controller.live.sailingState.currentHeading = -30.0f;
    controller.live.sailingState.desiredHeading = 30.0f;

    RudderSM_Update(&controller, 60.0f);

    ASSERT_EQ_INT(TACKING, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_enters_tacking_for_mirrored_upwind_signs(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 30.0f;
    controller.live.sailingState.desiredHeading = -30.0f;

    RudderSM_Update(&controller, -60.0f);

    ASSERT_EQ_INT(TACKING, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_stays_straight_for_same_side_upwind_headings(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 30.0f;
    controller.live.sailingState.desiredHeading = 45.0f;

    RudderSM_Update(&controller, -15.0f);

    ASSERT_EQ_INT(STRAIGHT, controller.live.stateMachine.currentState);
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

static bool test_state_machine_enters_gybing_for_mirrored_downwind_signs(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = -150.0f;
    controller.live.sailingState.desiredHeading = -210.0f;

    RudderSM_Update(&controller, 60.0f);

    ASSERT_EQ_INT(GYBING, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_stays_straight_for_same_side_downwind_headings(void)
{
    reset_test_controller();

    controller.live.windState.windDirection = 0.0f;
    controller.live.sailingState.currentHeading = 150.0f;
    controller.live.sailingState.desiredHeading = 120.0f;

    RudderSM_Update(&controller, 30.0f);

    ASSERT_EQ_INT(STRAIGHT, controller.live.stateMachine.currentState);
    return true;
}

static bool test_state_machine_blocks_low_wind_transition(void)
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
    ASSERT_TRUE(fake_tick_ms < controller.live.stateMachine.transitionGuards.timestampBlock[STRAIGHT][LOWWIND]);

    fake_tick_ms += 1001;
    RudderSM_Update(&controller, 0.0f);
    ASSERT_EQ_INT(LOWWIND, controller.live.stateMachine.currentState);
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
        {"state machine enters tacking for mirrored upwind signs", test_state_machine_enters_tacking_for_mirrored_upwind_signs},
        {"state machine stays straight for same-side upwind headings", test_state_machine_stays_straight_for_same_side_upwind_headings},
        {"state machine enters gybing", test_state_machine_enters_gybing},
        {"state machine enters gybing for mirrored downwind signs", test_state_machine_enters_gybing_for_mirrored_downwind_signs},
        {"state machine stays straight for same-side downwind headings", test_state_machine_stays_straight_for_same_side_downwind_headings},
        {"state machine blocks low wind transition", test_state_machine_blocks_low_wind_transition},
    };

    return run_tests(tests, sizeof(tests) / sizeof(tests[0]));
}
