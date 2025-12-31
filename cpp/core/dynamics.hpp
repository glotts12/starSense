#pragma once
#include "types.hpp"

namespace starSense {

class AttitudeDynamics {
public:
    virtual ~AttitudeDynamics() = default;

    // Compute time derivative of state: xdot = f(t, x, tau_body)
    virtual AttitudeState computeDerivative(
        double t,
        const AttitudeState &x,
        const Vec3 &tauBody   // control + disturbances, in body frame
    ) const = 0;
};

// Release 1: kinematic-only / free-omega dynamics (w_dot = 0)
class KinematicDynamics : public AttitudeDynamics {
public:
    AttitudeState computeDerivative(
        double t,
        const AttitudeState &x,
        const Vec3 &tauBody
    ) const override;
};

// Release 2+: rigid-body with inertia, real w_dot
class RigidBodyDynamics : public AttitudeDynamics {
public:
    explicit RigidBodyDynamics(const Mat3 &inertiaBody);

    AttitudeState computeDerivative(
        double t,
        const AttitudeState &x,
        const Vec3 &tauBody
    ) const override;

private:
    Mat3 inertia_;
    Mat3 inertiaInv_;
};

} // namespace starSense
