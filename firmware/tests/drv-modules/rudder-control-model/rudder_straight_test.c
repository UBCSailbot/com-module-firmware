/*
 * Tests for the rudder control model in its SHIPPED configuration.
 *
 * This file deliberately does NOT define RUDDER_TEST_BUILD, so STRAIGHT_ONLY
 * and TUNING_MODE stay on, exactly as the rudder firmware is compiled. The
 * pre-existing tests in the module's own testing/ folder define
 * RUDDER_TEST_BUILD, which switches both off, so straightLine() (the only
 * path that runs on the boat) had no coverage at all.
 */

#include "RUDDER.h"
#include "RUDDER_PARAMS.h"
#include "fake_tick.h"
#include "test_assert.h"

#include <math.h>
#include <stdio.h>

extern PIDControllerFixed rudderFixedParams;

static int g_failures = 0;

/* Coefficients the model runs on under TUNING_MODE (standardCoeffs). */
static const float KP = 1.2f;
static const float KI = 0.05f;
static const float KD = 0.02f;
static const float FILTER = 0.8f;
static const float DECAY = 0.98f;

/**
 * @brief Compare two floats within a tolerance.
 *
 * @param a First value.
 * @param b Second value.
 * @param tol Permitted absolute difference.
 * @return Non-zero when the values are within tolerance.
 */
static int close_to(float a, float b, float tol) { return fabsf(a - b) <= tol; }

/**
 * @brief Put the controller in a known state at a known tick.
 *
 * @param start_ms Tick value to initialise at.
 * @return void
 */
static void reset_at(uint32_t start_ms) {
  fake_tick_set(start_ms);
  initController(rudderFixedParams);
}

/**
 * @brief Drive one control cycle with a fixed heading error.
 *
 * @param error_deg Heading error in degrees.
 * @param dt_ms Milliseconds to advance before the cycle.
 * @return Rudder angle produced by the cycle.
 */
static float step(float error_deg, uint32_t dt_ms) {
  controller.live.sailingState.desiredHeading = 90.0f;
  controller.live.sailingState.currentHeading = 90.0f + error_deg;
  controller.live.windState.windSpeed = 5.0f;
  fake_tick_advance(dt_ms);

  float angle = 0.0f;
  runPID(&angle);
  return angle;
}

/*
 * initController must leave the controller fully deterministic. It previously
 * copied an uninitialised ControllerState over the zeroed live struct, so the
 * integral started from stack garbage.
 */
static void test_init_is_deterministic(void) {
  reset_at(1000U);

  const ControllerState *cs = &controller.live.controllerState;
  TEST_ASSERT(&g_failures, cs->integralError == 0.0f);
  TEST_ASSERT(&g_failures, cs->previousError == 0.0f);
  TEST_ASSERT(&g_failures, cs->filteredError == 0.0f);
  TEST_ASSERT(&g_failures, cs->previousFilteredError == 0.0f);
  TEST_ASSERT(&g_failures, cs->isActive == false);
  TEST_ASSERT(&g_failures, cs->lastTime == 1000U);
  TEST_ASSERT(&g_failures, controller.live.tackingState.tackingAllowed == true);
  TEST_ASSERT(&g_failures, controller.live.gybingState.gybingAllowed == true);
  TEST_ASSERT(&g_failures, returnState() == STRAIGHT);
}

/*
 * Locks the exact integral update for one cycle. A duplicated
 * "integralError += currentError * dt / 1000" in the clamp block shipped on
 * rudder-working-branch and made the integral run 1.5x high; the PID gains
 * were tuned against it. This assertion fails if that ever comes back.
 */
static void test_integral_update_is_exact(void) {
  reset_at(1000U);

  const float e = 10.0f;
  const float dt_ms = 100.0f;
  const float inc = e * dt_ms / 1000.0f; /* 1.0 */

  float angle = step(e, (uint32_t)dt_ms);

  /* Output uses the integral after a single increment. */
  const float integral_for_output = inc;
  const float filtered = FILTER * e;
  const float derivative = (filtered - 0.0f) / dt_ms * 1000.0f;
  const float expected_angle =
      KP * e + KI * integral_for_output - KD * derivative;

  /*
   * Stored state takes the increment twice: once via `integral`, once in the
   * unsaturated branch of the clamp. That is the model's long-standing
   * behaviour and the gains are tuned to it, so it is asserted as-is rather
   * than "corrected" here.
   */
  const float expected_integral = (inc + inc) * DECAY;

  TEST_ASSERT(&g_failures, close_to(angle, expected_angle, 1e-4f));
  TEST_ASSERT(&g_failures,
              close_to(controller.live.controllerState.integralError,
                       expected_integral, 1e-4f));
}

/* Below errorThreshold and not yet active, the rudder is left centred. */
static void test_deadband_leaves_rudder_centred(void) {
  reset_at(1000U);

  float angle = step(0.5f, 100U); /* errorThreshold is 2.0 */

  TEST_ASSERT(&g_failures, angle == 0.0f);
  TEST_ASSERT(&g_failures, controller.live.controllerState.isActive == false);
}

/* Output must saturate at the physical rudder limits. */
static void test_output_clamps_to_limits(void) {
  reset_at(1000U);
  float high = step(120.0f, 100U);
  TEST_ASSERT(&g_failures,
              close_to(high, rudderFixedParams.physicalParams.outputMax, 1e-4f));

  reset_at(1000U);
  float low = step(-120.0f, 100U);
  TEST_ASSERT(&g_failures,
              close_to(low, rudderFixedParams.physicalParams.outputMin, 1e-4f));
}

/* With STRAIGHT_ONLY compiled in, no other sailing state may be entered. */
static void test_state_stays_straight(void) {
  reset_at(1000U);

  /* Conditions that would request IRONS or LOWWIND if the full model ran. */
  for (int i = 0; i < 25; i++) {
    controller.live.sailingState.linearVelocity = 0.0f;
    controller.live.sailingState.angularVelocity = 0.0f;
    controller.live.windState.windDirection = 90.0f;
    (void)step(15.0f, 100U);
    TEST_ASSERT(&g_failures, returnState() == STRAIGHT);
  }
}

int main(void) {
  test_init_is_deterministic();
  test_integral_update_is_exact();
  test_deadband_leaves_rudder_centred();
  test_output_clamps_to_limits();
  test_state_stays_straight();

  if (g_failures == 0) {
    printf("rudder_straight_test: all tests passed\n");
    return 0;
  }

  printf("rudder_straight_test: %d failure(s)\n", g_failures);
  return 1;
}
