#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  
#include "api.hpp"

namespace py = pybind11;

PYBIND11_MODULE(starSense, m) {
    m.doc() = "StarSense attitude simulation bindings";

    py::class_<starSense::BallisticSimParams>(m, "BallisticSimParams")
        .def(py::init<>())
        .def_readwrite("q0", &starSense::BallisticSimParams::q0)
        .def_readwrite("w0", &starSense::BallisticSimParams::w0)
        .def_readwrite("dt", &starSense::BallisticSimParams::dt)
        .def_readwrite("numSteps", &starSense::BallisticSimParams::numSteps)
        .def_readwrite("integratorType", &starSense::BallisticSimParams::integratorType);

    py::class_<starSense::BallisticSimOutput>(m, "BallisticSimOutput")
        .def_readonly("time", &starSense::BallisticSimOutput::time)
        .def_readonly("quats", &starSense::BallisticSimOutput::quats)
        .def_readonly("omegas", &starSense::BallisticSimOutput::omegas);

    m.def("run_ballistic_simulation",
          &starSense::runBallisticSimulation,
          "Run a basic ballistic attitude simulation");
}
