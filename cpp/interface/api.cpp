#include "api.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "sensor.hpp"
#include "actuator.hpp"
#include "controller.hpp"

namespace starSense {

BallisticSimOutput runBallisticSimulation(const BallisticSimParams &params) {
    // Build dynamics
    auto dynamics = std::make_unique<KinematicDynamics>();

    // Build integrator
    IntegrationMethod method =
        (params.integratorType == "euler") ? IntegrationMethod::Euler
                                           : IntegrationMethod::RK4;
    auto integrator = std::make_unique<Integrator>(method);

    // Zero controller, ideal sensor & actuator
    auto controller = std::make_unique<ZeroController>();
    auto sensor     = std::make_unique<IdealAttitudeSensor>();
    auto actuator   = std::make_unique<IdealTorqueActuator>();

    // Reference: constant attitude equal to initial for now
    auto refProvider = [q0 = params.q0](double t) -> ReferenceProfile {
        ReferenceProfile ref;
        ref.qRef = q0;
        ref.wRef = {0.0, 0.0, 0.0};
        return ref;
    };

    AttitudeSimulation sim(
        std::move(dynamics),
        std::move(integrator),
        std::move(controller),
        std::move(sensor),
        std::move(actuator),
        refProvider
    );

    SimulationConfig cfg{0.0, params.dt, params.numSteps};
    AttitudeState x0{params.q0, params.w0};

    SimulationResult res = sim.run(cfg, x0);

    BallisticSimOutput out;
    out.time.reserve(res.time.size());
    out.quats.reserve(res.state.size());
    out.omegas.reserve(res.state.size());

    for (std::size_t i = 0; i < res.state.size(); ++i) {
        out.time.push_back(res.time[i]);
        out.quats.push_back(res.state[i].q);
        out.omegas.push_back(res.state[i].w);
    }
    return out;
}

} // namespace starSense
