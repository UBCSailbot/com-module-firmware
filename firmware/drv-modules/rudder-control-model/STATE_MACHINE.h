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
} SailingStateMachine;

typedef struct {
    float desired_heading_deg;
    bool tack_requested;
    bool gybe_requested;
    bool manual_mode;
} GuidanceCommand;

