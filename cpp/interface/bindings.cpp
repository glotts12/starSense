#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  
#include "api.hpp"

namespace py = pybind11;

PYBIND11_MODULE(starSense, m) {
    m.doc() = "StarSense attitude simulation bindings";

    // Simulation parameters
    py::class_<starSense::AttitudeSimParams>(m, "AttitudeSimParams")
        .def(py::init<>())
        .def_readwrite("q0", &starSense::AttitudeSimParams::q0)
        .def_readwrite("w0", &starSense::AttitudeSimParams::w0)
        .def_readwrite("inertiaBody", &starSense::AttitudeSimParams::inertiaBody)
        .def_readwrite("t0", &starSense::AttitudeSimParams::t0)
        .def_readwrite("dt", &starSense::AttitudeSimParams::dt)
        .def_readwrite("numSteps", &starSense::AttitudeSimParams::numSteps)
        .def_readwrite("integratorType", &starSense::AttitudeSimParams::integratorType)
        .def_readwrite("controllerType", &starSense::AttitudeSimParams::controllerType)
        .def_readwrite("referenceType", &starSense::AttitudeSimParams::referenceType)
        .def_readwrite("sensorType", &starSense::AttitudeSimParams::sensorType)
        .def_readwrite("actuatorType", &starSense::AttitudeSimParams::actuatorType);

    // Simulation output
    py::class_<starSense::AttitudeSimOutput>(m, "AttitudeSimOutput")
        .def_readonly("time", &starSense::AttitudeSimOutput::time)
        .def_readonly("quats", &starSense::AttitudeSimOutput::quats)
        .def_readonly("omegas", &starSense::AttitudeSimOutput::omegas)
        .def_readonly("commandedTorque", &starSense::AttitudeSimOutput::commandedTorque)
        .def_readonly("appliedTorque", &starSense::AttitudeSimOutput::appliedTorque);

    // Main entrypoint
    m.def(
        "run_simulation",
        &starSense::runSimulation,
        "Run a rigid-body attitude simulation"
    );
}
