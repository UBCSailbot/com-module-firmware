#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODE_STRAIGHT,
    MODE_TACKING,
    MODE_GYBING,
    MODE_LOW_WIND,
    MODE_IRONS,
    MODE_MANUAL,
    MODE_FAULT
} SailingMode;

typedef struct {
    SailingMode current_mode;
    SailingMode requested_mode;

    uint32_t mode_start_ms;
    uint32_t last_transition_ms;

    bool tack_blocked;
    bool gybe_blocked;

    uint32_t tack_block_until_ms;
    uint32_t gybe_block_until_ms;

    int32_t tack_cooldown_ms;
    int32_t gybe_cooldown_ms;

    float maneuver_target_heading_deg;
    float maneuver_initial_heading_deg;

    float low_wind_threshold;
    bool fault_detected;
} SailingStateMachine;

typedef struct {
    float average_heading_deg;
    float average_rel_wind_angle_deg;
    float boat_speed_mps;
    bool fault_detected;
} EstimatedBoatState;

typedef struct {
    float desired_heading_deg;
    bool tack_requested;
    bool gybe_requested;
    bool manual_mode;
} GuidanceCommand;

SailingMode SailingSM_Update(
    SailingStateMachine *sm,
    const EstimatedBoatState *state,
    const GuidanceCommand *cmd,
    uint32_t now_ms
);

bool isUpwind(const EstimatedBoatState *state);
bool isDownwind(const EstimatedBoatState *state);
bool is_in_irons(const EstimatedBoatState *state);

#endif /* STATE_MACHINE_H_ */

