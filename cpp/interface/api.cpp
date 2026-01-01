#include "api.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "sensor.hpp"
#include "actuator.hpp"
#include "controller.hpp"

namespace starSense {

// Build controller from params
std::unique_ptr<Controller> makeController(
    const std::string &controllerType) {
    if (controllerType == "zero") {
        return std::make_unique<ZeroController>();
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

AttitudeSimOutput runSimulation(const AttitudeSimParams &params) {
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
    auto controller = makeController(params.controllerType);

    // Build sensor
    auto sensor = makeSensor(params.sensorType);

    // Build actuator
    auto actuator = makeActuator(params.actuatorType);

    // Reference: constant attitude equal to initial for now
    auto refProvider = [q0 = params.q0](double t) -> ReferenceProfile {
        ReferenceProfile ref;
        ref.qRef = q0;
        ref.wRef = {0.0, 0.0, 0.0};
        return ref;
    };

    // Construct simulation object
    AttitudeSimulation sim(
        std::move(dynamics),
        std::move(integrator),
        std::move(controller),
        std::move(sensor),
        std::move(actuator),
        refProvider
    );

    SimulationConfig cfg{params.t0, params.dt, params.numSteps};
    AttitudeState x0{params.q0, params.w0};

    // Run the simulation
    SimulationResult simResult = sim.run(cfg, x0);

    // Pack the output
    AttitudeSimOutput out;
    out.time            = simResult.time;
    out.commandedTorque = simResult.commandedTorque;
    out.appliedTorque   = simResult.appliedTorque;

    out.quats.reserve(simResult.state.size());
    out.omegas.reserve(simResult.state.size());
    for (const auto &s : simResult.state) {
        out.quats.push_back(s.q);
        out.omegas.push_back(s.w);
    }

    return out;
}

} // namespace starSense
