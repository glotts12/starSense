#include "dynamics.hpp"

namespace starSense {

AttitudeState KinematicDynamics::computeDerivative(
    double t,
    const AttitudeState &x,
    const Vec3 &tauBody
) const {
    (void)t;       // no explicit time dependence in this model
    (void)tauBody; // no torque 

    const Quat &q = x.q;  // [q0, q1, q2, q3], scalar first
    const Vec3 &w = x.w;  // [wx, wy, wz] in body frame

    const double wx = w[0];
    const double wy = w[1];
    const double wz = w[2];

    AttitudeState xdot;

    // Quaternion kinematics: q_dot = 0.5 * Ω(ω) * q
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

RigidBodyDynamics::RigidBodyDynamics(const Mat3 &inertiaBody)
    : J_(inertiaBody),
      Jinv_(inverse(inertiaBody))
{ }

AttitudeState RigidBodyDynamics::computeDerivative(
    double t,
    const AttitudeState& x,
    const Vec3& tauBody
) const {
    (void)t; // no explicit time dependence in this model

    const Quat &q = x.q;  // [q0, q1, q2, q3], scalar first
    const Vec3 &w = x.w;  // [wx, wy, wz] in body frame

    const double wx = w[0];
    const double wy = w[1];
    const double wz = w[2];

    AttitudeState xdot;

    // Quaternion kinematics: q_dot = 0.5 * Ω(ω) * q
    xdot.q[0] = 0.5 * (-wx * q[1] - wy * q[2] - wz * q[3]);
    xdot.q[1] = 0.5 * ( wx * q[0] + wz * q[2] - wy * q[3]);
    xdot.q[2] = 0.5 * ( wy * q[0] - wz * q[1] + wx * q[3]);
    xdot.q[3] = 0.5 * ( wz * q[0] + wy * q[1] - wx * q[2]);

    // Rigid-body dynamics: wdot = J^{-1} ( tauBody - w × (J w) )
    Vec3 Jw = matmul(J_, x.w);       // J * w
    Vec3 wxJw = cross(x.w, Jw);      // w × (J w)
    Vec3 rhs = sub(tauBody, wxJw);   // RHS = tauBody - w × (J w)
    Vec3 wdot = matmul(Jinv_, rhs);  // wdot = Jinv * rhs
    xdot.w = wdot;

    return xdot;
}

} // namespace starSense

