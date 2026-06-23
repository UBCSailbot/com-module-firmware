#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "STATE_MACHINE.h"
#include "RUDDER_PARAMS.h"

static float wrap180(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static float abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

static const bool rudderAllowedTransitions[COUNT][COUNT] = {
/* FROM:      TO:  STRAIGHT   TACKING   GYBING    LOWWIND  IRONS  MANUAL  */
/* STRAIGHT */ { true,        true,     true,     true,     true,    true },
/* TACKING  */ { true,        false,    false,    true,     true,    true },
/* GYBING   */ { true,        false,    false,    true,     true,    true },
/* LOWWIND  */ { true,        true,     true,     true,     true,    true },
/* IRONS    */ { true,        false,    false,    true,     true,    true },
/* MANUAL   */ { true,        false,    false,    true,     true,    true }
};

static void requestState(StateMachine *state_machine, State next)
{
    if (state_machine == NULL) {
        return;
    }

    state_machine->nextState = next;
}

static void updateStateMachine(StateMachine *state_machine)
{
    if (state_machine == NULL) {
        return;
    }

    State from = state_machine->currentState;
    State to = state_machine->nextState;

    if (from == to) {
        return;
    }

    if (from >= COUNT || to >= COUNT) {
        return;
    }

    if (!rudderAllowedTransitions[from][to]) {
        return;
    }

    if (!time_reached(
            HAL_GetTick(),
            state_machine->transitionGuards.timestampBlock[from][to]
        )) {
        return;
    }

    state_machine->currentState = to;
    state_machine->lastTransition = HAL_GetTick();
}

static bool isTackingCondition(PIDController *controller, float error)
{
    const volatile WindState *wind = &controller->live.windState;
    const volatile SailingState *sailing = &controller->live.sailingState;
    PIDcoefficients *pid = &controller->fixed.tackingCoeffs;

    float desired_wind_angle =
        wrap180(wind->windDirection - sailing->desiredHeading);
    float boat_wind_angle =
        wrap180(wind->windDirection - sailing->currentHeading);

    if (!controller->live.tackingState.tackingAllowed) {
        return false;
    }

    if (controller->live.tackingState.isTacking) {
        uint32_t tack_duration_ms =
            (uint32_t)(controller->fixed.scalingCoeffs.tackTime * 1000.0f);

        if (HAL_GetTick() - controller->live.tackingState.tackingStartTime <
            tack_duration_ms) {
            return true;
        }

        controller->live.tackingState.isTacking = false;
        return false;
    }

    if (abs_float(error) <= pid->headingTolerance) {
        return false;
    }

    return abs_float(boat_wind_angle) < 90.0f &&
           abs_float(desired_wind_angle) < 90.0f &&
           ((desired_wind_angle > 0.0f && boat_wind_angle < 0.0f) ||
            (desired_wind_angle < 0.0f && boat_wind_angle > 0.0f));
}

static bool isGybingCondition(PIDController *controller, float error)
{
    volatile WindState *wind = &controller->live.windState;
    volatile SailingState *sailing = &controller->live.sailingState;
    PIDcoefficients *pid = &controller->fixed.gybingCoeffs;

    float desired_wind_angle =
        wrap180(wind->windDirection - sailing->desiredHeading);
    float boat_wind_angle =
        wrap180(wind->windDirection - sailing->currentHeading);

    if (!controller->live.gybingState.gybingAllowed) {
        return false;
    }

    if (controller->live.gybingState.isGybing) {
        uint32_t gybe_duration_ms =
            (uint32_t)(controller->fixed.scalingCoeffs.gybeTime * 1000.0f);

        if (HAL_GetTick() - controller->live.gybingState.gybingStartTime <
            gybe_duration_ms) {
            return true;
        }

        controller->live.gybingState.isGybing = false;
        return false;
    }

    if (abs_float(error) <= pid->headingTolerance) {
        return false;
    }

    return abs_float(boat_wind_angle) > 90.0f &&
           abs_float(desired_wind_angle) > 90.0f &&
           ((desired_wind_angle > 0.0f && boat_wind_angle < 0.0f) ||
            (desired_wind_angle < 0.0f && boat_wind_angle > 0.0f));
}

void RudderSM_Init(StateMachine *state_machine)
{
    if (state_machine == NULL) {
        return;
    }

    state_machine->currentState = STRAIGHT;
    state_machine->nextState = STRAIGHT;
    state_machine->lastTransition = HAL_GetTick();
    memset(
        state_machine->transitionGuards.timestampBlock,
        0,
        sizeof(state_machine->transitionGuards.timestampBlock)
    );
}

void RudderSM_BlockTransition(
    TransitionGuards *guards,
    State from,
    State to,
    uint32_t duration_ms
)
{
    if (guards == NULL || from >= COUNT || to >= COUNT) {
        return;
    }

    guards->timestampBlock[from][to] = HAL_GetTick() + duration_ms;
}

void RudderSM_Update(PIDController *controller, float error)
{
    if (controller == NULL) {
        return;
    }

    PhysicalParams *params = &controller->fixed.physicalParams;
    volatile WindState *wind = &controller->live.windState;

#ifdef STRAIGHT_ONLY
    requestState(&controller->live.stateMachine, STRAIGHT);
    updateStateMachine(&controller->live.stateMachine);
    return;
#endif

    if (isTackingCondition(controller, error)) {
        requestState(&controller->live.stateMachine, TACKING);
    } else if (isGybingCondition(controller, error)) {
        requestState(&controller->live.stateMachine, GYBING);
    } else if (wind->windSpeed < params->lowWindThreshold) {
        requestState(&controller->live.stateMachine, LOWWIND);
    } else if (RudderSM_IsInIrons(controller)) {
        requestState(&controller->live.stateMachine, IRONS);
    } else {
        requestState(&controller->live.stateMachine, STRAIGHT);
    }

    updateStateMachine(&controller->live.stateMachine);
}

bool RudderSM_IsInIrons(const PIDController *controller)
{
    if (controller == NULL) {
        return false;
    }

    volatile WindState *wind = &controller->live.windState;
    volatile SailingState *sailing = &controller->live.sailingState;
    const StateThresholds *thresholds = &controller->fixed.stateThresholds;
    const PhysicalParams *params = &controller->fixed.physicalParams;
    float wind_angle =
        abs_float(wrap180(wind->windDirection - sailing->currentHeading));
    float upwind_range =
        params->upwindIronsRange > 0.0f ?
        params->upwindIronsRange :
        params->upwindIronsAngle;
    float downwind_range =
        params->downwindIronsRange > 0.0f ?
        params->downwindIronsRange :
        params->downwindIronsAngle;
    bool low_speed =
        abs_float(sailing->linearVelocity) <= thresholds->ironsSpeed;
    bool low_rotation =
        abs_float(sailing->angularVelocity) <= thresholds->stateironsRot;
    bool pointed_into_wind = wind_angle <= upwind_range;
    bool pointed_dead_downwind =
        abs_float(wind_angle - 180.0f) <= downwind_range;

    return low_speed &&
           low_rotation &&
           (pointed_into_wind || pointed_dead_downwind);
}

static void enter_mode(
    SailingStateMachine *sm,
    SailingMode new_mode,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static SailingMode evaluate_from_straight(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static SailingMode evaluate_tacking(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static SailingMode evaluate_gybing(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static SailingMode evaluate_low_wind(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static SailingMode evaluate_irons(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

static bool transition_allowed(
    SailingStateMachine *sm,
    SailingMode requested,
    uint32_t now_ms
) {
    static const bool allowed[MODE_FAULT + 1][MODE_FAULT + 1] = {
        [MODE_STRAIGHT] = {
            [MODE_STRAIGHT] = true,
            [MODE_TACKING] = true,
            [MODE_GYBING] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_TACKING] = {
            [MODE_STRAIGHT] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_GYBING] = {
            [MODE_STRAIGHT] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_LOW_WIND] = {
            [MODE_STRAIGHT] = true,
            [MODE_TACKING] = true,
            [MODE_GYBING] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_IRONS] = {
            [MODE_STRAIGHT] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_MANUAL] = {
            [MODE_STRAIGHT] = true,
            [MODE_LOW_WIND] = true,
            [MODE_IRONS] = true,
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
        [MODE_FAULT] = {
            [MODE_MANUAL] = true,
            [MODE_FAULT] = true,
        },
    };

    if (sm == NULL) {
        return false;
    }

    if (sm->current_mode > MODE_FAULT || requested > MODE_FAULT) {
        return false;
    }

    if (!allowed[sm->current_mode][requested]) {
        return false;
    }

    if (requested == MODE_TACKING) {
        if (sm->tack_blocked && !time_reached(now_ms, sm->tack_block_until_ms)) {
            return false;
        }

        sm->tack_blocked = false;
    }

    if (requested == MODE_GYBING) {
        if (sm->gybe_blocked && !time_reached(now_ms, sm->gybe_block_until_ms)) {
            return false;
        }

        sm->gybe_blocked = false;
    }

    return true;
}

SailingMode SailingSM_Update(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
) {
    SailingMode requested = sm->current_mode;

    switch(sm->current_mode) {
        case MODE_STRAIGHT:
            requested = evaluate_from_straight(sm, state, cmd, now_ms);
            break;

        case MODE_TACKING:
            requested = evaluate_tacking(sm, state, cmd, now_ms);
            break;

        case MODE_GYBING:
            requested = evaluate_gybing(sm, state, cmd, now_ms);
            break;

        case MODE_LOW_WIND:
            requested = evaluate_low_wind(sm, state, cmd, now_ms);
            break;

        case MODE_IRONS:
            requested = evaluate_irons(sm, state, cmd, now_ms);
            break;

        default:
            requested = MODE_STRAIGHT;
            break;
    }

    if(requested != sm->current_mode && transition_allowed(sm, requested, now_ms)) {
        enter_mode(sm, requested, state, cmd, now_ms);
    }

    return sm->current_mode;
}

static void enter_mode(
    SailingStateMachine *sm,
    SailingMode new_mode,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    SailingMode old_mode = sm->current_mode;

    sm->current_mode = new_mode;
    sm->mode_start_ms = now_ms;

    if (old_mode == MODE_TACKING) {
        sm->tack_blocked = true;
        sm->tack_block_until_ms = now_ms + sm->tack_cooldown_ms;
    }

    if (old_mode == MODE_GYBING) {
        sm->gybe_blocked = true;
        sm->gybe_block_until_ms = now_ms + sm->gybe_cooldown_ms;
    }
}

bool isUpwind(const EstimatedBoatState *state)
{
    if (state == NULL) {
        return false;
    }

    float average_wind_direction_deg =
        state->average_heading_deg + state->average_rel_wind_angle_deg;
    float average_rel_wind_angle_deg =
        wrap180(average_wind_direction_deg - state->average_heading_deg);

    return abs_float(average_rel_wind_angle_deg) < 90.0f;
}

bool isDownwind(const EstimatedBoatState *state)
{
    if (state == NULL) {
        return false;
    }

    float average_wind_direction_deg =
        state->average_heading_deg + state->average_rel_wind_angle_deg;
    float average_rel_wind_angle_deg =
        wrap180(average_wind_direction_deg - state->average_heading_deg);

    return abs_float(average_rel_wind_angle_deg) > 90.0f;
}

bool is_in_irons(const EstimatedBoatState *state)
{
    if (state == NULL) {
        return false;
    }

    PIDControllerFixed fixed_params = getRudderFixedParams();
    float rel_wind_angle_deg = wrap180(state->average_rel_wind_angle_deg);

    if (isUpwind(state)) {
        return abs_float(rel_wind_angle_deg) <= fixed_params.physicalParams.upwindIronsRange;
    }

    else {
        return abs_float(abs_float(rel_wind_angle_deg) - 180.0f) <= fixed_params.physicalParams.downwindIronsRange;
    }
}

static SailingMode evaluate_from_straight(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    if (cmd->manual_mode) {
        return MODE_MANUAL;
    }

    if (state->fault_detected) {
        return MODE_FAULT;
    }

    if (is_in_irons(state)) {
        return MODE_IRONS;
    }

    if (state->boat_speed_mps < sm->low_wind_threshold) {
        return MODE_LOW_WIND;
    }

    if (cmd->tack_requested) {
        return MODE_TACKING;
    }

    if (cmd->gybe_requested) {
        return MODE_GYBING;
    }

    return MODE_STRAIGHT;
}

static SailingMode evaluate_tacking(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    if (cmd->manual_mode) {
        return MODE_MANUAL;
    }

    if (state->fault_detected) {
        return MODE_FAULT;
    }

    if (is_in_irons(state)) {
        return MODE_IRONS;
    }

    if (state->boat_speed_mps < sm->low_wind_threshold) {
        return MODE_LOW_WIND;
    }

    if (abs_float(wrap180(state->average_heading_deg - cmd->desired_heading_deg)) <=
        getRudderFixedParams().tackingCoeffs.headingTolerance) {
        return MODE_STRAIGHT;
    }

    return MODE_TACKING;
}

static SailingMode evaluate_gybing(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    if (cmd->manual_mode) {
        return MODE_MANUAL;
    }

    if (state->fault_detected) {
        return MODE_FAULT;
    }

    if (is_in_irons(state)) {
        return MODE_IRONS;
    }

    if (state->boat_speed_mps < sm->low_wind_threshold) {
        return MODE_LOW_WIND;
    }

    if (abs_float(wrap180(state->average_heading_deg - cmd->desired_heading_deg)) <=
        getRudderFixedParams().gybingCoeffs.headingTolerance) {
        return MODE_STRAIGHT;
    }

    return MODE_GYBING;
}

static SailingMode evaluate_low_wind(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    if (cmd->manual_mode) {
        return MODE_MANUAL;
    }

    if (state->fault_detected) {
        return MODE_FAULT;
    }

    if (is_in_irons(state)) {
        return MODE_IRONS;
    }

    if (state->boat_speed_mps < sm->low_wind_threshold) {
        return MODE_LOW_WIND;
    }

    if (cmd->tack_requested) {
        return MODE_TACKING;
    }

    if (cmd->gybe_requested) {
        return MODE_GYBING;
    }

    return MODE_STRAIGHT;
}

static SailingMode evaluate_irons(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
)
{
    if (cmd->manual_mode) {
        return MODE_MANUAL;
    }

    if (state->fault_detected) {
        return MODE_FAULT;
    }

    if (is_in_irons(state)) {
        return MODE_IRONS;
    }

    if (state->boat_speed_mps < sm->low_wind_threshold) {
        return MODE_LOW_WIND;
    }

    return MODE_STRAIGHT;
}
