#pragma once
#include "types.hpp"
#include "util.hpp"

namespace starSense {

class AttitudeDynamics {
public:
    virtual ~AttitudeDynamics() = default;

    // compute time derivative of state: xdot = f(t, x, tau_body)
    virtual AttitudeState computeDerivative(
        double t,
        const AttitudeState &x,
        const Vec3 &tauBody   // control + disturbances, in body frame
    ) const = 0;
};

// kinematic-only / free-omega dynamics (w_dot = 0)
class KinematicDynamics : public AttitudeDynamics {
public:
    AttitudeState computeDerivative(
        double t,
        const AttitudeState &x,
        const Vec3 &tauBody
    ) const override;
};

// rigid-body with inertia, real w_dot
class RigidBodyDynamics : public AttitudeDynamics {
public:
    explicit RigidBodyDynamics(const Mat3& inertiaBody);

    AttitudeState computeDerivative(
        double t,
        const AttitudeState& x,
        const Vec3& tauBody
    ) const override;

private:
    Mat3 J_;     // inertia matrix in body frame
    Mat3 Jinv_;  // its inverse
};

} // namespace starSense
