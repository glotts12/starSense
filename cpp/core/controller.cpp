#include "controller.hpp"

namespace starSense {

// ZeroController
Vec3 ZeroController::computeCommandTorque(
    double t,
    const AttitudeState &estimatedState,
    const ReferenceProfile &reference
) const {
    (void)t;
    (void)estimatedState;
    (void)reference;

    // No control: torque is identically zero
    return Vec3{0.0, 0.0, 0.0};
}

// PDController (rate-only placeholder)
PDController::PDController(double kpAtt, double kdRate)
    : kpAtt_(kpAtt),
      kdRate_(kdRate) { }

Vec3 PDController::computeCommandTorque(
    double t,
    const AttitudeState &estimatedState,
    const ReferenceProfile &reference
) const {
    (void)t;
    (void)kpAtt_; // attitude gain not used yet

    Vec3 torque{0.0, 0.0, 0.0};

    // Simple rate-damping control:
    //   tau = -kd * (w - w_ref)
    for (std::size_t i = 0; i < 3; ++i) {
        double rateErr = estimatedState.w[i] - reference.wRef[i];
        torque[i] = -kdRate_ * rateErr;
    }

    return torque;
}

} // namespace starSense
