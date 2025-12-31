#include <simulation.hpp>

namespace starSense {

AttitudeSimulation::AttitudeSimulation(
    std::unique_ptr<AttitudeDynamics> dynamics,
    std::unique_ptr<Integrator> integrator,
    std::unique_ptr<Controller> controller,
    std::unique_ptr<Sensor> sensor,
    std::unique_ptr<Actuator> actuator,
    std::function<ReferenceProfile(double)> referenceProvider
)
    : dynamics_(std::move(dynamics)),
      integrator_(std::move(integrator)),
      controller_(std::move(controller)),
      sensor_(std::move(sensor)),
      actuator_(std::move(actuator)),
      referenceProvider_(std::move(referenceProvider))
{}

SimulationResult AttitudeSimulation::run(
    const SimulationConfig &cfg,
    const AttitudeState &x0
) const {
    SimulationResult result;

    // Reserve memory
    const int nSteps = cfg.numSteps;
    result.time.reserve(nSteps + 1);
    result.state.reserve(nSteps + 1);
    result.commandedTorque.reserve(nSteps);
    result.appliedTorque.reserve(nSteps);

    const double t0 = cfg.t0;
    const double dt = cfg.dt;

    // torqueFunc: sensor -> controller -> actuator -> tau_body
    //
    // this lambda is called by the Integrator once per step.
    // It:
    //   1. "measures" the state via the Sensor
    //   2. gets reference attitude/rate from the referenceProvider_
    //   3. computes commanded torque via Controller
    //   4. maps that to applied torque via Actuator
    //   5. logs both commanded & applied torque into `result`
    //
    auto torqueFunc = [this, &result](double t, const AttitudeState &x) -> Vec3 {
        // 1. sensor measurement (for now: ideal attitude sensor)
        Quat qMeas = sensor_->measureAttitude(t, x);

        // estimated state = true state but with measured attitude
        AttitudeState estimatedState = x;
        estimatedState.q = qMeas;

        // 2. reference profile (desired attitude / rate at time t)
        ReferenceProfile ref = referenceProvider_(t);

        // 3. controller: compute commanded torque in body frame
        Vec3 commanded = controller_->computeCommandTorque(t, estimatedState, ref);

        // 4. actuator: apply command, get actual applied torque
        Vec3 applied = actuator_->applyCommand(t, x, commanded);

        // 5. log torques for this step
        result.commandedTorque.push_back(commanded);
        result.appliedTorque.push_back(applied);

        // this is what the dynamics sees
        return applied;
    };

    // integrate dynamics
    std::vector<AttitudeState> stateHistory = integrator_->integrate(
        *dynamics_,
        t0,
        x0,
        dt,
        nSteps,
        torqueFunc
    );

    // populate time and state history in result
    for (int k = 0; k <= nSteps; ++k) {
        double tk = t0 + static_cast<double>(k) * dt;
        result.time.push_back(tk);
        result.state.push_back(stateHistory[k]);
    }

    return result;
}

}