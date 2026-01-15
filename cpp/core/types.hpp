#pragma once
#include <array>

namespace starSense {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // [q0, q1, q2, q3], scalar first
using Mat3 = std::array<std::array<double, 3>, 3>;
using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat3x6 = std::array<std::array<double, 6>, 3>;

struct AttitudeState {
    Quat q;   // unit quaternion, body wrt inertial
    Vec3 w;   // angular rate in body frame [rad/s]
};

} // namespace starSense
