#include "actuator.hpp"
#include <stdexcept>
#include <algorithm>

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


ReactionWheelActuator::ReactionWheelActuator(
    const std::vector<Vec3>& wheelAxes,
    const std::vector<double>& wheelInertias,
    const std::vector<double>& maxTorque,
    const std::vector<double>& maxSpeed,
    const std::vector<double>& initialSpeeds
)
    : wheelAxes_(wheelAxes)
    , wheelInertias_(wheelInertias)
    , maxTorque_(maxTorque)
    , maxSpeedRPM_(maxSpeed)
    , wheelSpeeds_(initialSpeeds)
    , lastTime_(-1.0)
{
    size_t n = wheelAxes_.size();
    if (wheelInertias_.size() != n ||
        maxTorque_.size() != n ||
        maxSpeedRPM_.size() != n ||
        wheelSpeeds_.size() != n) {
        throw std::invalid_argument(
            "ReactionWheelActuator: all wheel parameter vectors must have the same size");
    }

    // Normalize wheel axes
    for (auto& axis : wheelAxes_) {
        double norm = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if (norm > 1e-10) {
            axis[0] /= norm;
            axis[1] /= norm;
            axis[2] /= norm;
        }
    }
}

Vec3 ReactionWheelActuator::applyCommand(
    double t,
    const AttitudeState &state,
    const Vec3 &command
) const {
    (void)state;  // not used currently

    // Compute time step (first call uses dt=0)
    double dt = (lastTime_ < 0.0) ? 0.0 : (t - lastTime_);
    lastTime_ = t;

    // Project commanded torque onto each wheel axis to get per-wheel torque commands
    // Then apply saturation and speed limits
    Vec3 appliedTorque = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < wheelAxes_.size(); ++i) {
        const Vec3& axis = wheelAxes_[i];

        // Project commanded torque onto this wheel's axis
        double cmdTorqueWheel = -(command[0]*axis[0] + command[1]*axis[1] + command[2]*axis[2]);

        // Apply torque saturation
        double saturatedTorque = std::clamp(cmdTorqueWheel, -maxTorque_[i], maxTorque_[i]);

        // Update wheel speed: tau = I * alpha => alpha = tau / I
        // omega_new = omega_old + alpha * dt
        if (dt > 0.0 && wheelInertias_[i] > 1e-12) {
            double alphaRads = saturatedTorque / wheelInertias_[i];  // rad/s²
            double deltaSpeedRPM = alphaRads * RADS_TO_RPM * dt;
            wheelSpeeds_[i] += deltaSpeedRPM;

            // Apply speed saturation
            wheelSpeeds_[i] = std::clamp(wheelSpeeds_[i], -maxSpeedRPM_[i], maxSpeedRPM_[i]);
        }

        // The reaction torque on the spacecraft is opposite to the wheel torque
        // tau_spacecraft = -tau_wheel (along axis)
        appliedTorque[0] += saturatedTorque * axis[0];
        appliedTorque[1] += saturatedTorque * axis[1];
        appliedTorque[2] += saturatedTorque * axis[2];
    }

    return appliedTorque;
}

} // namespace starSense
