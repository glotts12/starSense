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
        .def_readwrite("dt", &starSense::AttitudeSimParams::dt)
        .def_readwrite("numSteps", &starSense::AttitudeSimParams::numSteps)
        .def_readwrite("integratorType", &starSense::AttitudeSimParams::integratorType)
        .def_readwrite("controllerType", &starSense::AttitudeSimParams::controllerType)
        .def_readwrite("kpAtt", &starSense::AttitudeSimParams::kpAtt)
        .def_readwrite("kdRate", &starSense::AttitudeSimParams::kdRate)
        .def_readwrite("kLqr", &starSense::AttitudeSimParams::kLqr)
        .def_readwrite("controlRateHz", &starSense::AttitudeSimParams::controlRateHz)
        .def_readwrite("wRef", &starSense::AttitudeSimParams::wRef)
        .def_readwrite("qRef", &starSense::AttitudeSimParams::qRef)
        .def_readwrite("referenceType", &starSense::AttitudeSimParams::referenceType)
        .def_readwrite("sensorType", &starSense::AttitudeSimParams::sensorType)
        .def_readwrite("actuatorType", &starSense::AttitudeSimParams::actuatorType);

    // Simulation Result
    py::class_<starSense::SimulationResult>(m, "SimulationResult")
        .def_readonly("time",            &starSense::SimulationResult::time)
        .def_readonly("quats",           &starSense::SimulationResult::quats)
        .def_readonly("omegas",          &starSense::SimulationResult::omegas)
        .def_readonly("commandedTorque", &starSense::SimulationResult::commandedTorque)
        .def_readonly("appliedTorque",   &starSense::SimulationResult::appliedTorque)
        // new fields, optional for Python users:
        .def_readonly("qRef",            &starSense::SimulationResult::qRef)
        .def_readonly("wRef",            &starSense::SimulationResult::wRef)
        .def_readonly("attitudeError",   &starSense::SimulationResult::attitudeError)
        .def_readonly("rateError",       &starSense::SimulationResult::rateError);

    // Main entrypoint
    m.def(
        "run_simulation",
        &starSense::runSimulation,
        "Run a rigid-body attitude simulation"
    );
}
