/*ESTIMATOR.H*/

typedef struct {
    float heading;
    float yawRate;
    float avgYawRate;

    float relWindAngle;
    float avgRelWindAngle;

    float windSpeed;
    float avgWindSpeed;

    float linearVel;
    float heelAngle;

    uint32_t lastUpdateMs;
    float prevHeading;

    bool initialized;
    bool fault_detected;
} StateEstimate;

void updateStateEstimate(
    StateEstimate *est,
    float measuredHeadingDeg,
    float measuredRelWindAngleDeg,
    float measuredWindSpeed,
    float measuredLinearVelocity,
    float measuredHeelAngleDeg
);

void applyEstimateToController(Controller *controller, StateEstimate *est);
