#include "controller.hpp"

namespace starSense {

// ZeroController
Vec3 ZeroController::computeCommandTorque(
    double t,
    const AttitudeState &estimatedState,
    const ReferenceState ref
) const {
    (void)t;
    (void)estimatedState;
    (void)ref;

    // No control: torque is identically zero
    return Vec3{0.0, 0.0, 0.0};
}

// PDController 
PDController::PDController(Vec3 kpAtt, Vec3 kdRate, double controlRateHz)
    : kpAtt_(kpAtt),
      kdRate_(kdRate),
      controlRateHz_(controlRateHz) { }

Vec3 PDController::computeCommandTorque(
    double t,
    const AttitudeState &estimatedState,
    const ReferenceState ref
) const {
    // If controlRateHz_ <= 0, update every call (no sample/hold behavior).
    const bool useSampleHold = (controlRateHz_ > 0.0);

    if (!useSampleHold || t >= nextUpdateTime_) {
        // Compute attitude error
        Quat qRefConj = quatConjugate(ref.qRef);
        Quat qErr     = quatMultiply(qRefConj, estimatedState.q);

        double qw = qErr[0];
        Vec3 qv   = { qErr[1], qErr[2], qErr[3] };

        double sign_qw = (qw >= 0.0) ? 1.0 : -1.0;

        // e_att ≈ rotation vector (small-angle) with shortest-rotation convention
        Vec3 eAtt = {
            2.0 * sign_qw * qv[0],
            2.0 * sign_qw * qv[1],
            2.0 * sign_qw * qv[2]
        };

        // Rate error: e_ω = ω - ω_ref
        Vec3 eW = {
            estimatedState.w[0] - ref.wRef[0],
            estimatedState.w[1] - ref.wRef[1],
            estimatedState.w[2] - ref.wRef[2]
        };

        // PD torque (per axis):
        Vec3 torque{0.0, 0.0, 0.0};
        for (std::size_t i = 0; i < 3; ++i) {
            torque[i] = -kpAtt_[i] * eAtt[i] - kdRate_[i] * eW[i];
        }

        // Store and schedule next update if using sample/hold
        lastTorque_ = torque;
        if (useSampleHold) {
            const double dtControl = 1.0 / controlRateHz_;
            nextUpdateTime_ = t + dtControl;
        }

        return torque;
    }

    // Between control updates: hold previous command
    return lastTorque_;
}

} // namespace starSense
