#include "util.hpp"

namespace starSense {

// Vector normalization

Vec3 normalize(const Vec3 &v) {
    double norm2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    if (norm2 == 0.0) {
        // Degenerate case – return original to avoid NaNs
        return v;
    }
    double invNorm = 1.0 / std::sqrt(norm2);
    return Vec3{v[0] * invNorm, v[1] * invNorm, v[2] * invNorm};
}

Quat normalize(const Quat &q) {
    double norm2 =
        q[0]*q[0] +
        q[1]*q[1] +
        q[2]*q[2] +
        q[3]*q[3];

    if (norm2 == 0.0) {
        // Degenerate – return identity quaternion
        return Quat{1.0, 0.0, 0.0, 0.0};
    }

    double invNorm = 1.0 / std::sqrt(norm2);
    return Quat{
        q[0] * invNorm,
        q[1] * invNorm,
        q[2] * invNorm,
        q[3] * invNorm
    };
}

// Basic vector ops

double dot(const Vec3 &a, const Vec3 &b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

Vec3 add(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2]
    };
}

Vec3 sub(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2]
    };
}

// 3x3 matrix ops

Mat3 transpose(const Mat3 &A) {
    Mat3 AT{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            AT[i][j] = A[j][i];
        }
    }
    return AT;
}

Mat3 matmul(const Mat3 &A, const Mat3 &B) {
    Mat3 C{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double acc = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                acc += A[i][k] * B[k][j];
            }
            C[i][j] = acc;
        }
    }
    return C;
}

Vec3 matmul(const Mat3 &A, const Vec3 &v) {
    Vec3 out{};
    for (std::size_t i = 0; i < 3; ++i) {
        out[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
    }
    return out;
}

Vec3 matmul(const Vec3 &v, const Mat3 &A) {
    Vec3 out{};
    for (std::size_t j = 0; j < 3; ++j) {
        out[j] = v[0]*A[0][j] + v[1]*A[1][j] + v[2]*A[2][j];
    }
    return out;
}

Mat3 inverse(const Mat3 &A) {
    // Compute determinant
    double det =
          A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
        - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
        + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (std::abs(det) < 1e-15) {
        throw std::runtime_error("inverse(Mat3): matrix is singular (det ~ 0)");
    }

    double invDet = 1.0 / det;

    Mat3 inv{};

    inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

    inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

    inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return inv;
}

void validateInertia(const Mat3 &J,
                     double symmetryTol,
                     double posDefTol) {
    // Symmetry check: J must equal J^T within symmetryTol
    if (std::fabs(J[0][1] - J[1][0]) > symmetryTol ||
        std::fabs(J[0][2] - J[2][0]) > symmetryTol ||
        std::fabs(J[1][2] - J[2][1]) > symmetryTol) {

        std::ostringstream oss;
        oss << "Inertia matrix must be symmetric. Got:\n"
            << "[" << J[0][0] << " " << J[0][1] << " " << J[0][2] << "; "
            <<  J[1][0] << " " << J[1][1] << " " << J[1][2] << "; "
            <<  J[2][0] << " " << J[2][1] << " " << J[2][2] << "]";
        throw std::invalid_argument(oss.str());
    }

    // Leading principal minors (Sylvester's criterion for SPD)
    const double m1 = J[0][0];

    const double m2 = J[0][0] * J[1][1] - J[0][1] * J[1][0];

    const double det =
        J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
        J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
        J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);

    if (m1 <= posDefTol || m2 <= posDefTol || det <= posDefTol) {
        std::ostringstream oss;
        oss << "Inertia matrix must be symmetric positive definite. "
            << "Leading minors: m1=" << m1
            << ", m2=" << m2
            << ", det=" << det;
        throw std::invalid_argument(oss.str());
    }
}

} // namespace starSense
