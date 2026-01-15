#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "types.hpp"
#include "simulation.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "sensor.hpp"
#include "actuator.hpp"
#include "controller.hpp"
#include "util.hpp"
#include "referenceProfile.hpp"

namespace starSense {

struct AttitudeSimParams {
    // Initial state
    Quat q0;      // [w, x, y, z]
    Vec3 w0;      // [wx, wy, wz] in body frame [rad/s]

    // Spacecraft properties
    Mat3 inertiaBody{  // full 3x3 inertia matrix in body frame - default unit cube
        std::array<double,3>{1.0, 0.0, 0.0},
        std::array<double,3>{0.0, 1.0, 0.0},
        std::array<double,3>{0.0, 0.0, 1.0}
    }; 

    // Time setup
    double dt = 0.1;         // step [s]
    int    numSteps = 1000;  // number of steps

    // Integrator
    std::string integratorType = "rk4";   // "euler" or "rk4"

    // Controller selection
    std::string controllerType = "zero";                // "zero", "pd", and "lqr" supported
    Vec3 kpAtt = std::array<double,3>{1.0, 1.0, 1.0};   // defaults
    Vec3 kdRate = std::array<double,3>{1.0, 1.0, 1.0};  // defaults
    Mat3x6 kLqr = {{                                    // defaults
        {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
        {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
        {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}
    }};
    double controlRateHz = dt;

    // Sensor selection
    std::string sensorType = "ideal";     // only "ideal" is supported right now

    // Actuator selection
    std::string actuatorType = "ideal";   // only "ideal" is supported right now

    // Reference profile selection
    std::string referenceType = "fixed";  // only fixed is supported right now
    Quat qRef;
    Vec3 wRef;
};

// Single, general entrypoint
SimulationResult runSimulation(const AttitudeSimParams &params);

} // namespace starSense
