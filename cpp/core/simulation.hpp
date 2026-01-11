#pragma once
#include <memory>
#include <vector>
#include <functional>

#include "types.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "sensor.hpp"
#include "actuator.hpp"
#include "controller.hpp"

namespace starSense {

struct SimulationConfig {
    double dt;
    int numSteps;
};

struct SimulationResult {
    std::vector<double> time;            // size N+1
    std::vector<Quat>   quats;           // size N+1
    std::vector<Vec3>   omegas;          // size N+1
    std::vector<Vec3>   commandedTorque; // size N
    std::vector<Vec3>   appliedTorque;   // size N

    // reference & error logs (size N+1)
    std::vector<Quat> qRef;
    std::vector<Vec3> wRef;
    std::vector<Vec3> attitudeError;   // 3-vector rotation error in body
    std::vector<Vec3> rateError;       // ω − ω_ref in body
};

class AttitudeSimulation {
public:
    AttitudeSimulation(
        std::unique_ptr<AttitudeDynamics> dynamics,
        std::unique_ptr<Integrator> integrator,
        std::unique_ptr<Controller> controller,
        std::unique_ptr<Sensor> sensor,
        std::unique_ptr<Actuator> actuator,
        std::unique_ptr<ReferenceProfile> referenceProfile
    );

    SimulationResult run(
        const SimulationConfig &cfg,
        const AttitudeState &x0
    ) const;

private:
    std::unique_ptr<AttitudeDynamics> dynamics_;
    std::unique_ptr<Integrator> integrator_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<Sensor> sensor_;
    std::unique_ptr<Actuator> actuator_;
    std::unique_ptr<ReferenceProfile> referenceProfile_;
};

} // namespace starSense
