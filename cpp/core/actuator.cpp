#include "actuator.hpp"

namespace starSense {

Vec3 IdealTorqueActuator::applyCommand(
    double t,
    const AttitudeState &state,
    const Vec3 &command
) const {
    (void)t;     // unused in ideal model
    (void)state; // unused in ideal model

    // Ideal actuator: applied torque = commanded torque
    return command;
}

} // namespace starSense
