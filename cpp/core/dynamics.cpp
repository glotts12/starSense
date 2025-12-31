#include "dynamics.hpp"

namespace starSense {

AttitudeState KinematicDynamics::computeDerivative(
    double t,
    const AttitudeState &x,
    const Vec3 &tauBody
) const {
    (void)t;       // unused in kinematic-only model
    (void)tauBody; // no torque 

    const Quat &q = x.q;  // [q0, q1, q2, q3], scalar first
    const Vec3 &w = x.w;  // [wx, wy, wz] in body frame

    const double wx = w[0];
    const double wy = w[1];
    const double wz = w[2];

    AttitudeState xdot;

    // Quaternion kinematics: q_dot = 0.5 * Ω(ω) * q
    // Ω(ω) =
    // [  0   -wx   -wy   -wz ]
    // [ wx    0    wz   -wy ]
    // [ wy   -wz    0    wx ]
    // [ wz    wy  -wx     0 ]
    //
    // q = [q0, q1, q2, q3]^T (scalar first)

    xdot.q[0] = 0.5 * (-wx * q[1] - wy * q[2] - wz * q[3]);
    xdot.q[1] = 0.5 * ( wx * q[0] + wz * q[2] - wy * q[3]);
    xdot.q[2] = 0.5 * ( wy * q[0] - wz * q[1] + wx * q[3]);
    xdot.q[3] = 0.5 * ( wz * q[0] + wy * q[1] - wx * q[2]);

    // Kinematic-only model: ω_dot = 0
    xdot.w[0] = 0.0;
    xdot.w[1] = 0.0;
    xdot.w[2] = 0.0;

    return xdot;
}

} // namespace starSense

