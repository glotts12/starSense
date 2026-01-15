#pragma once

#include "util.hpp"
#include "referenceProfile.hpp"

namespace starSense {

// Abstract controller interface
class Controller {
public:
    virtual ~Controller() = default;

    // Compute commanded body-frame torque [N·m]
    //  t              : current simulation time [s]
    //  estimatedState : estimated attitude state (q, w)
    //  reference      : desired attitude profile (qRef, wRef)
    virtual Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceState ref
    ) const = 0;
};


// Zero controller
class ZeroController : public Controller {
public:
    Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceState ref
    ) const override;
};


// PD controller
class PDController : public Controller {
public:
    PDController(Vec3 kpAtt, Vec3 kdRate, double controlRateHz);

    Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceState ref
    ) const override;

private:
    Vec3 kpAtt_;                              // attitude gain
    Vec3 kdRate_;                             // rate damping gain
    double controlRateHz_;                    // how often to update control command
    mutable double nextUpdateTime_ = 0.0;     // next time to refresh torque
    mutable Vec3 lastTorque_{0.0, 0.0, 0.0};  // held command between updates};
};

// Linear Quadratic Regulator (LQR) controller
class LQRController : public Controller {
public:
    LQRController(const Mat3x6 &K, double controlRateHz);

    Vec3 computeCommandTorque(
        double t,
        const AttitudeState &estimatedState,
        const ReferenceState ref
    ) const override;

private:
    Mat3x6 K_;                                // 3x6 gain matrix passes in from Python
    double controlRateHz_;                    // how often to update control command
    mutable double nextUpdateTime_ = 0.0;     // next time to refresh torque
    mutable Vec3 lastTorque_{0.0, 0.0, 0.0};  // held command between updates};
};

} // namespace starSense
