#pragma once

#include "types.hpp"
#include <vector>
#include <cmath>

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


// Reaction wheel actuator with saturation limits
class ReactionWheelActuator : public Actuator {
public:
    // wheelAxes: spin axis for each wheel in body frame (should be normalized)
    // wheelInertias: moment of inertia about spin axis for each wheel [kg·m²]
    // maxTorque: max torque each wheel can apply [N·m]
    // maxSpeed: max wheel speed [RPM]
    // initialSpeeds: initial wheel speeds [RPM]
    ReactionWheelActuator(
        const std::vector<Vec3>& wheelAxes,
        const std::vector<double>& wheelInertias,
        const std::vector<double>& maxTorque,
        const std::vector<double>& maxSpeed,
        const std::vector<double>& initialSpeeds
    );

    Vec3 applyCommand(
        double t,
        const AttitudeState &state,
        const Vec3 &command
    ) const override;

    // Get current wheel speeds [RPM]
    const std::vector<double>& getWheelSpeeds() const { return wheelSpeeds_; }

private:
    std::vector<Vec3> wheelAxes_;
    std::vector<double> wheelInertias_;
    std::vector<double> maxTorque_;
    std::vector<double> maxSpeedRPM_;
    mutable std::vector<double> wheelSpeeds_;  // mutable for const applyCommand
    mutable double lastTime_;

    static constexpr double RPM_TO_RADS = M_PI / 30.0;
    static constexpr double RADS_TO_RPM = 30.0 / M_PI;
};

} // namespace starSense
