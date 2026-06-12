#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "STATE_MACHINE.h"

static bool time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return (int32_t)(now_ms - target_ms) >= 0;
}

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
