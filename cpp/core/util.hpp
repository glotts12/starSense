#pragma once

#include <cstddef>
#include <cmath>
#include <algorithm>
#include <sstream>

#include "types.hpp"

namespace starSense {

// ------------------------------
// Vector normalization
// ------------------------------
Vec3 normalize(const Vec3 &v);
Quat normalize(const Quat &q);

// ------------------------------
// Basic vector ops
// ------------------------------
double dot(const Vec3 &a, const Vec3 &b);
Vec3   cross(const Vec3 &a, const Vec3 &b);

// Component-wise add/sub
Vec3 add(const Vec3 &a, const Vec3 &b);
Vec3 sub(const Vec3 &a, const Vec3 &b);

// ------------------------------
// 3x3 matrix ops
// ------------------------------
Mat3 transpose(const Mat3 &A);

// A * B  (3x3 * 3x3)
Mat3 matmul(const Mat3 &A, const Mat3 &B);

// A * v  (3x3 * 3x1)
Vec3 matmul(const Mat3 &A, const Vec3 &v);

// v * A  (1x3 * 3x3) – treat v as row vector
Vec3 matmul(const Vec3 &v, const Mat3 &A);

// 3x3 inverse (throws if singular)
Mat3 inverse(const Mat3 &A);

} // namespace starSense
