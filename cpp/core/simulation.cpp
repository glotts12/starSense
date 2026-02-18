#include <simulation.hpp>

namespace starSense {

AttitudeSimulation::AttitudeSimulation(
    std::unique_ptr<AttitudeDynamics> dynamics,
    std::unique_ptr<Integrator> integrator,
    std::unique_ptr<Controller> controller,
    std::unique_ptr<Sensor> sensor,
    std::unique_ptr<Actuator> actuator,
    std::unique_ptr<ReferenceProfile> referenceProfile
)
    : dynamics_(std::move(dynamics)),
      integrator_(std::move(integrator)),
      controller_(std::move(controller)),
      sensor_(std::move(sensor)),
      actuator_(std::move(actuator)),
      referenceProfile_(std::move(referenceProfile))
{}

SimulationResult AttitudeSimulation::run(
    const SimulationConfig &cfg,
    const AttitudeState &x0
) const {
    SimulationResult result;
    const int nSteps = cfg.numSteps;    
    const double t0 = 0;  // always start from t = 0
    const double dt = cfg.dt;

    // Reserve memory
    result.time.reserve(nSteps + 1);
    result.commandedTorque.reserve(nSteps);
    result.appliedTorque.reserve(nSteps);
    result.qRef.reserve(nSteps + 1);
    result.wRef.reserve(nSteps + 1);
    result.attitudeError.reserve(nSteps + 1);
    result.rateError.reserve(nSteps + 1);

    // torqueFunc: sensor -> controller -> actuator -> tau_body
    auto torqueFunc = [this, &result](double t, const AttitudeState &x) -> Vec3 {
        // 1. sensor measurement 
        Quat qMeas = sensor_->measureAttitude(t, x);

        // estimated state = true state but with measured attitude
        AttitudeState estimatedState = x;
        estimatedState.q = qMeas; 

        // 2. reference state (desired attitude / rate at time t)
        ReferenceState ref = referenceProfile_->computeReferenceState(t);

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

    // populate time, state, and extended logs
    for (int k = 0; k <= nSteps; ++k) {
        double tk      = t0 + static_cast<double>(k) * dt;
        const auto &xk = stateHistory[k];

        // state
        result.time.push_back(tk);
        result.quats.push_back(xk.q);
        result.omegas.push_back(xk.w);

        // reference at this grid time
        ReferenceState ref = referenceProfile_->computeReferenceState(tk);
        result.qRef.push_back(ref.qRef);
        result.wRef.push_back(ref.wRef);

        // attitude error: q_err = q_ref^{-1} ⊗ q
        Quat qRefConj = quatConjugate(ref.qRef);
        Quat qErr     = quatMultiply(qRefConj, xk.q);

        double qw = qErr[0];
        Vec3 qv   = { qErr[1], qErr[2], qErr[3] };

        double sign_qw = (qw >= 0.0) ? 1.0 : -1.0;

        Vec3 eAtt = {
            2.0 * sign_qw * qv[0],
            2.0 * sign_qw * qv[1],
            2.0 * sign_qw * qv[2]
        };

        // rate error: ω − ω_ref
        Vec3 eW = {
            xk.w[0] - ref.wRef[0],
            xk.w[1] - ref.wRef[1],
            xk.w[2] - ref.wRef[2]
        };

        result.attitudeError.push_back(eAtt);
        result.rateError.push_back(eW);
    }

    return result;
}

}