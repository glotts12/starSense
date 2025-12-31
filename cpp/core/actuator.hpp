#pragma once

#include "types.hpp"

namespace starSense {

// Abstract actuator interface.
// Takes a commanded body-frame torque and returns the actual applied torque.
class Actuator {
public:
    virtual ~Actuator() = default;

    // t         : current simulation time [s]
    // state     : current attitude state (q, w)
    // command   : commanded torque in body frame [N·m]
    // returns   : applied torque in body frame [N·m]
    virtual Vec3 applyCommand(
        double t,
        const AttitudeState &state,
        const Vec3 &command
    ) const = 0;
};


// Release 1: ideal actuator (no saturation, no dynamics, no noise)
class IdealTorqueActuator : public Actuator {
public:
    Vec3 applyCommand(
        double t,
        const AttitudeState &state,
        const Vec3 &command
    ) const override;
};

} // namespace starSense
