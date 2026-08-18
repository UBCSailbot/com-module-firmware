/*
 * Tests for the rudder state machine transition guards.
 *
 * Built with RUDDER_TEST_BUILD so STRAIGHT_ONLY is off and transitions can
 * actually be exercised.
 */

#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
#include "STATE_MACHINE.h"
#include "fake_tick.h"
#include "test_assert.h"

#include <stdio.h>

extern PIDControllerFixed rudderFixedParams;

static int g_failures = 0;

/*
 * An unset guard is stored as 0 and must always read as expired. The guard
 * check uses a signed difference so it survives tick rollover, but that
 * reports a 0 guard as still pending once the tick counter passes 2^31 ms
 * (24.9 days of uptime), which froze every transition.
 */
static void test_zero_guard_never_blocks(void) {
  const uint32_t past_rollover = 3000000000U; /* ~34.7 days */

  fake_tick_set(past_rollover);
  initController(rudderFixedParams);

  StateMachine *sm = &controller.live.stateMachine;
  TEST_ASSERT(&g_failures, sm->currentState == STRAIGHT);

  /* No guard was ever set, so a permitted transition must go through. */
  RudderSM_Update(&controller, 0.0f);
  controller.live.windState.windSpeed = 0.0f; /* below lowWindThreshold */
  RudderSM_Update(&controller, 0.0f);

  TEST_ASSERT(&g_failures, sm->currentState == LOWWIND);
}

/* A guard that was explicitly set must still block until it expires. */
static void test_set_guard_blocks_until_expiry(void) {
  fake_tick_set(1000U);
  initController(rudderFixedParams);

  StateMachine *sm = &controller.live.stateMachine;
  RudderSM_BlockTransition(&sm->transitionGuards, STRAIGHT, LOWWIND, 5000U);

  controller.live.windState.windSpeed = 0.0f;

  fake_tick_set(3000U); /* still inside the block window */
  RudderSM_Update(&controller, 0.0f);
  TEST_ASSERT(&g_failures, sm->currentState == STRAIGHT);

  fake_tick_set(6001U); /* past the block window */
  RudderSM_Update(&controller, 0.0f);
  TEST_ASSERT(&g_failures, sm->currentState == LOWWIND);
}

/* Transitions the table forbids must not happen. */
static void test_forbidden_transition_is_rejected(void) {
  fake_tick_set(1000U);
  initController(rudderFixedParams);

  StateMachine *sm = &controller.live.stateMachine;
  sm->currentState = TACKING;
  sm->nextState = TACKING;

  /* TACKING -> GYBING is false in the transition table. */
  controller.live.windState.windSpeed = 5.0f;
  sm->nextState = GYBING;
  RudderSM_Update(&controller, 0.0f);

  TEST_ASSERT(&g_failures, sm->currentState != GYBING);
}

int main(void) {
  test_zero_guard_never_blocks();
  test_set_guard_blocks_until_expiry();
  test_forbidden_transition_is_rejected();

  if (g_failures == 0) {
    printf("rudder_state_machine_test: all tests passed\n");
    return 0;
  }

  printf("rudder_state_machine_test: %d failure(s)\n", g_failures);
  return 1;
}
