/*ESTIMATOR.H*/

typedef struct {
    float heading;
    float yawRate;
    float avgYawRate;

    float relWindAngle;
    float avgRelWindAngle;

    float linearVel;
    float heelAngle;

    uint32_t lastUpdateMs;
    float prevHeading;

    bool initialized;
} StateEstimate;

void updateStateEstimate(
    StateEstimate *est,
    float measuredHeadingDeg,
    float measuredRelWindAngleDeg,
    float measuredLinearVelocity,
    float measuredHeelAngleDeg
)

void applyEstimateToController(Controller *controller, StateEstimate *est)