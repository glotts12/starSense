#include "sensor.hpp"

namespace starSense {

Quat IdealAttitudeSensor::measureAttitude(
    double t,
    const AttitudeState &trueState
) const {
    (void)t;  // unused in ideal model

    // Perfect measurement: return true attitude
    return trueState.q;
}

} // namespace starSense
