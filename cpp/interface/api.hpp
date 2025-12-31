#pragma once
#include <vector>
#include <string>
#include "types.hpp"
#include "simulation.hpp"

namespace starSense {

struct BallisticSimParams {
    Quat q0;
    Vec3 w0;
    double dt;
    int numSteps;
    std::string integratorType; // "euler" or "rk4"
};

struct BallisticSimOutput {
    std::vector<double> time;
    std::vector<Quat> quats;
    std::vector<Vec3> omegas;
};

// High-level function used by Python
BallisticSimOutput runBallisticSimulation(const BallisticSimParams &params);

} // namespace starSense
