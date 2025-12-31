#include "integrator.hpp"

namespace starSense {

Integrator::Integrator(IntegrationMethod method)
    : method_(method) {}

// Euler step 
AttitudeState Integrator::stepEuler_(
    const AttitudeDynamics &dyn,
    double t,
    const AttitudeState &x,
    double dt,
    const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
) const {
    // Single torque sample per step (also logs via torqueFunc)
    Vec3 tau = torqueFunc(t, x);

    // State derivative
    AttitudeState xdot = dyn.computeDerivative(t, x, tau);

    // Forward Euler update
    AttitudeState xNext;

    // Quaternion components
    for (std::size_t i = 0; i < 4; ++i) {
        xNext.q[i] = x.q[i] + dt * xdot.q[i];
    }

    // Angular rate components
    for (std::size_t i = 0; i < 3; ++i) {
        xNext.w[i] = x.w[i] + dt * xdot.w[i];
    }

    // Enforce unit quaternion
    xNext.q = normalize(xNext.q);

    return xNext;
}

// RK4 step 
AttitudeState Integrator::stepRK4_(
    const AttitudeDynamics &dyn,
    double t,
    const AttitudeState &x,
    double dt,
    const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
) const {
    // Sample torque *once* at the start of the step.
    // We hold it constant over the step (good enough for now).
    Vec3 tau = torqueFunc(t, x);

    // k1
    AttitudeState k1 = dyn.computeDerivative(t, x, tau);

    // x + dt/2 * k1
    AttitudeState xTemp;
    for (std::size_t i = 0; i < 4; ++i) {
        xTemp.q[i] = x.q[i] + 0.5 * dt * k1.q[i];
    }
    for (std::size_t i = 0; i < 3; ++i) {
        xTemp.w[i] = x.w[i] + 0.5 * dt * k1.w[i];
    }
    AttitudeState k2 = dyn.computeDerivative(t + 0.5 * dt, xTemp, tau);

    // x + dt/2 * k2
    for (std::size_t i = 0; i < 4; ++i) {
        xTemp.q[i] = x.q[i] + 0.5 * dt * k2.q[i];
    }
    for (std::size_t i = 0; i < 3; ++i) {
        xTemp.w[i] = x.w[i] + 0.5 * dt * k2.w[i];
    }
    AttitudeState k3 = dyn.computeDerivative(t + 0.5 * dt, xTemp, tau);

    // x + dt * k3
    for (std::size_t i = 0; i < 4; ++i) {
        xTemp.q[i] = x.q[i] + dt * k3.q[i];
    }
    for (std::size_t i = 0; i < 3; ++i) {
        xTemp.w[i] = x.w[i] + dt * k3.w[i];
    }
    AttitudeState k4 = dyn.computeDerivative(t + dt, xTemp, tau);

    // Combine stages
    AttitudeState xNext;
    for (std::size_t i = 0; i < 4; ++i) {
        xNext.q[i] = x.q[i] + (dt / 6.0) *
            (k1.q[i] + 2.0 * k2.q[i] + 2.0 * k3.q[i] + k4.q[i]);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        xNext.w[i] = x.w[i] + (dt / 6.0) *
            (k1.w[i] + 2.0 * k2.w[i] + 2.0 * k3.w[i] + k4.w[i]);
    }

    // Normalize quaternion to maintain unit norm
    xNext.q = normalize(xNext.q);

    return xNext;
}

// Propagation loop 
std::vector<AttitudeState> Integrator::integrate(
    const AttitudeDynamics &dynamics,
    double t0,
    const AttitudeState &x0,
    double dt,
    int numSteps,
    const std::function<Vec3(double, const AttitudeState&)> &torqueFunc
) const {
    std::vector<AttitudeState> states;
    states.reserve(static_cast<std::size_t>(numSteps) + 1);

    double t = t0;
    AttitudeState x = x0;

    states.push_back(x);

    for (int k = 0; k < numSteps; ++k) {
        switch (method_) {
        case IntegrationMethod::Euler:
            x = stepEuler_(dynamics, t, x, dt, torqueFunc);
            break;
        case IntegrationMethod::RK4:
            x = stepRK4_(dynamics, t, x, dt, torqueFunc);
            break;
        default:
            x = stepRK4_(dynamics, t, x, dt, torqueFunc);
            break;
        }

        t += dt;
        states.push_back(x);
    }

    return states;
}

} // namespace starSense