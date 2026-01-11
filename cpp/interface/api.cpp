#include "api.hpp"

namespace starSense {

// Validate that an inertia matrix is symmetric and postive definite
void validateInertia(const Mat3 &J,
                     double symmetryTol=1e-10,
                     double posDefTol=1e-12) {
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

// Check for valid timestep
void validateTimestep(const AttitudeSimParams& params) {
    double wx = params.w0[0];
    double wy = params.w0[1];
    double wz = params.w0[2];
    double wNorm = std::sqrt(wx*wx + wy*wy + wz*wz);

    double h = wNorm * params.dt; // rad per step

    if (h > 1.0) {
        std::cerr
            << "WARNING (starSense): integration time step too large for given angular rate.\n"
            << "    |w0| = " << wNorm << " rad/s, dt = " << params.dt
            << " s -> |w0|*dt = " << h << " rad/step (~"
            << (h * 180.0 / M_PI) << " deg/step).\n"
            << "    Expect poor accuracy. "
            << "Consider reducing dt or w0.\n";
    } else if (h > 0.3) {
        std::cerr
            << "NOTE (starSense): integration coarse step for given angular rate.\n"
            << "    |w0|*dt = " << h << " rad/step (~"
            << (h * 180.0 / M_PI) << " deg/step).\n"
            << "    Results may be low accuracy over long durations.\n";
    }
}

// Build controller from params
std::unique_ptr<Controller> makeController(
    const std::string &controllerType, Vec3 kpAtt, Vec3 kdRate, double controlRateHz) {
    if (controllerType == "zero") {
        return std::make_unique<ZeroController>();
    } else if (controllerType == "pd") {
        return std::make_unique<PDController>(kpAtt, kdRate, controlRateHz);
    } else {
        throw std::invalid_argument(
            "runSimulation: unsupported controllerType = " + controllerType);
    }
}

// Build sensor from params
std::unique_ptr<Sensor> makeSensor(const std::string &sensorType) {
    if (sensorType == "ideal") {
        return std::make_unique<IdealAttitudeSensor>();
    } else {
        throw std::invalid_argument(
            "runSimulation: unsupported sensorType = " + sensorType
        );
    }
}

// Build actuator from params
std::unique_ptr<Actuator> makeActuator(const std::string &actuatorType) {
    if (actuatorType == "ideal") {
        return std::make_unique<IdealTorqueActuator>();
    } else {
        throw std::invalid_argument(
            "runSimulation: unsupported actuatorType = " + actuatorType
        );
    }
}

// build the reference profile from params
std::unique_ptr<ReferenceProfile> makeReferenceProfile(
    const std::string& referenceType,
    const Quat& qRef,
    const Vec3& wRef
) {
    if (referenceType == "fixed") {
        return std::make_unique<ConstantReferenceProfile>(qRef, wRef);
    } else {
        throw std::invalid_argument(
            "runSimulation: unsupported referenceType = " + referenceType
        );
    }
}

SimulationResult runSimulation(const AttitudeSimParams &params) {
    // Validate inputs
    validateInertia(params.inertiaBody);
    validateTimestep(params);  // NOTE: In the future switch to an adaptive step integrator

    // Build dynamics
    auto dynamics = std::make_unique<RigidBodyDynamics>(params.inertiaBody);

    // Build integrator
    IntegrationMethod method;
    if (params.integratorType == "euler") {
        method = IntegrationMethod::Euler;
    } else if (params.integratorType == "rk4") {
        method = IntegrationMethod::RK4;
    } else {
        throw std::invalid_argument(
            "runSimulation: unsupported integratorType = " + params.integratorType);
    }
    auto integrator = std::make_unique<Integrator>(method);

    // Build controller
    auto controller = makeController(params.controllerType, params.kpAtt, params.kdRate, params.controlRateHz);

    // Build sensor
    auto sensor = makeSensor(params.sensorType);

    // Build actuator
    auto actuator = makeActuator(params.actuatorType);

    // Reference: constant attitude equal to initial for now
    auto refProvider = makeReferenceProfile(params.referenceType, params.qRef, params.wRef);

    // Construct simulation object
    AttitudeSimulation sim(
        std::move(dynamics),
        std::move(integrator),
        std::move(controller),
        std::move(sensor),
        std::move(actuator),
        std::move(refProvider)
    );

    SimulationConfig cfg{params.dt, params.numSteps};
    AttitudeState x0{params.q0, params.w0};

    // Run the simulation
    SimulationResult simResult = sim.run(cfg, x0);

    return simResult;
}

} // namespace starSense
