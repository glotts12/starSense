#pragma once
#include <vector>
#include <functional>
#include "types.hpp"
#include "dynamics.hpp"
#include "util.hpp"

namespace starSense {

enum class IntegrationMethod {
    Euler,
    RK4
};

class Integrator {
public:
    explicit Integrator(IntegrationMethod method);

    // Integrate from t0, x0 forward with fixed step dt
    std::vector<AttitudeState> integrate(
        const AttitudeDynamics &dynamics,
        double t0,
        const AttitudeState &x0,
        double dt,
        int numSteps,
        const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
    ) const;

private:
    IntegrationMethod method_;

    AttitudeState stepEuler_(
        const AttitudeDynamics &dyn,
        double t,
        const AttitudeState &x,
        double dt,
        const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
    ) const;

    AttitudeState stepRK4_(
        const AttitudeDynamics &dyn,
        double t,
        const AttitudeState &x,
        double dt,
        const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
    ) const;
};

} // namespace starSense
