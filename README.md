# starSense 🛰️

**starSense** is a small C++/Python sandbox for rigid–body spacecraft attitude simulation.

- C++ core: dynamics, integration, control, sensors, actuators  
- Python: bindings (pybind11) + Plotly visualizations  
- Use it to play with attitude dynamics, controls, and visualization without dragging an entire flight dynamics stack around.

---

## 1. Project Overview

Current capabilities:

- Single rigid body with constant inertia matrix **J** in the body frame
- State:
  - Quaternion attitude `q = [w, x, y, z]`
  - Angular velocity `ω` in body frame
- Dynamics:
  - Quaternion kinematics:  
    \[
    \dot{q} = \frac{1}{2}\,\Omega(\omega)\,q
    \]
  - Rigid–body rotational dynamics:  
    \[
    J\dot{\omega} + \omega \times (J \omega) = \tau_b
    \]
- Reference profile:
  - Fixed reference attitude `qRef` and angular rate `wRef`
- Controller:
  - PD controller in body frame:
    \[
    \tau_b = -K_p e_{\text{att}} - K_d e_\omega
    \]
    with  
    \( q_{\text{err}} = q_{\text{ref}}^{-1} \otimes q \),  
    \( e_{\text{att}} = 2\,\mathrm{sgn}(q_{0,\text{err}}) \, q_{v,\text{err}} \),  
    \( e_\omega = \omega - \omega_{\text{ref}} \).
  - Sample-and-hold at a user-specified control rate (Hz)
- Sensors & actuators:
  - Ideal attitude “sensor” (no noise yet)
  - Ideal actuator (commanded torque = applied torque)
- Python visualizations:
  - Quaternion components vs time
  - Euler angles vs time
  - 3D animated body axes
  - Rotational kinetic energy
  - Attitude error and rate error (components + norms)

The C++ library is exposed to Python as a module named `starSense` via pybind11.

See `starSense/python/run.py` for an example simulation.

---

## 2. Repository Layout

```text
starSense/
├── CMakeLists.txt
├── cpp
│   ├── core
│   │   ├── actuator.hpp / actuator.cpp
│   │   ├── controller.hpp / controller.cpp
│   │   ├── dynamics.hpp / dynamics.cpp
│   │   ├── integrator.hpp / integrator.cpp
│   │   ├── sensor.hpp / sensor.cpp
│   │   ├── simulation.hpp / simulation.cpp
│   │   ├── types.hpp
│   │   ├── util.hpp / util.cpp
│   └── interface
│       ├── api.hpp / api.cpp        # run_simulation(...) API
│       └── bindings.cpp             # pybind11 module definition
├── python
│   ├── attitude_plotting.py         # Plotly visualization utilities
│   ├── run.py                       # Example script
├── requirements.txt