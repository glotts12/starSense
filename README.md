# starSense 🛰️

**starSense** is a small C++/Python sandbox for rigid-body spacecraft attitude simulation.

- **C++ core:** dynamics, integration, control, sensors, actuators  
- **Python:** pybind11 bindings + Plotly visualizations  
- Goal: a lightweight playground to explore attitude dynamics and control laws (PD, LQR, etc.) without dragging a full flight-dynamics stack around.

---

## 1. Project Overview

Current capabilities:

- **Rigid-body attitude dynamics**
  - Single spacecraft with a constant inertia matrix `J` in the body frame
  - State:
    - Quaternion attitude `q = [w, x, y, z]`
    - Angular velocity `ω` in the body frame
  - Dynamics:
    - Quaternion kinematics
    - Rigid-body rotational dynamics with `J`, `ω`, and body-frame torque `τ_b`

- **Reference profiles**
  - Fixed reference attitude `qRef`
  - Spinning, Earth-pointing, nadir-pointing, velocity-aligned, etc. coming soon ...

- **Controllers**
  - **Zero controller**
    - No control torque (pure ballistic / free-tumble)
  - **PD controller**
    - Attitude error from a quaternion error (shortest-rotation convention)
    - Rate error `ω − ω_ref`
    - Diagonal gains `Kp`, `Kd`
    - Sample-and-hold at a user-specified control rate (Hz)
  - **LQR controller**
    - Linearized attitude + rate error state
    - Gains `K` generated in Python from user-supplied Q/R weights and inertia
    - Same sample-and-hold infrastructure as PD

- **Sensors & actuators**
  - Ideal attitude “sensor” (no noise or bias yet)
  - Ideal actuator (commanded torque = applied torque)
  - Sensor/Actuator + Noise and uncertainty coming soon ...

- **Space environment modeling**
  - Coming soon ... 

- **Python tooling**
  - `starSense` Python module (via pybind11)
  - Plotly-based visualization utilities
  - Example scripts for PD and LQR controlled simulations

---

## 2. Repository Layout

```text
starSense/
├── CMakeLists.txt
├── cpp
│   ├── core
│   │   ├── actuator.hpp / actuator.cpp      # actuator models (ideal for now)
│   │   ├── controller.hpp / controller.cpp  # Zero, PD, LQR controllers
│   │   ├── dynamics.hpp / dynamics.cpp      # kinematic + rigid-body dynamics
│   │   ├── integrator.hpp / integrator.cpp  # Euler / RK4 integration
│   │   ├── sensor.hpp / sensor.cpp          # attitude "sensor" models
│   │   ├── simulation.hpp / simulation.cpp  # AttitudeSimulation driver
│   │   ├── types.hpp                        # Vec3, Quat, etc.
│   │   ├── util.hpp / util.cpp              # math helpers (quats, matrices)
│   └── interface
│       ├── api.hpp / api.cpp                # run_simulation(...) API
│       └── bindings.cpp                     # pybind11 module definition
├── python
│   ├── attitude_plotting.py                 # Plotly visualization utilities
│   ├── lqr_utils.py                         # LQR gain builder (Q/R -> K)
│   ├── run_pd_controls.py                   # Example: PD-controlled sim
│   ├── run_lqr_controls.py                  # Example: LQR-controlled sim
├── requirements.txt                         # Python deps (pybind11, plotly, etc.)
```

## 3. Building and Running

### 3.1 Prerequisites

Make sure you have the following installed:

- **Python 3.10**  
  > The project is currently wired to Python 3.10 on macOS.
- **C++17 compiler**  
  (e.g., AppleClang, `g++`, `clang++`)
- **CMake ≥ 3.15**
- **pip** (for installing Python dependencies)

#### Recommended Python Environment Setup

From the **repo root**:

```bash
python3 -m venv .venv
source .venv/bin/activate   # or equivalent on your system

pip install --upgrade pip
pip install -r requirements.txt
```

---

### 3.2 Build the C++ Extension

From the **repo root**:

```bash
mkdir -p build
cd build
cmake ..
cmake --build .
cd ..
```

This produces a shared module which can be imported directly by Python:
```
build/starSense.so
```

---

### 3.3 Run the Examples

From the **repo root**:

```bash
PYTHONPATH=build python python/run_pd_controls.py
```

You can similarly run the LQR example:

```bash
PYTHONPATH=build python python/run_lqr_controls.py
```

---

## 4. Outputs & Plotting

`starSense.run_simulation(params)` returns a C++ `SimulationResult` exposed to Python with fields including:

- `time` – time history (N+1 samples)
- `state` – internal state objects (quaternions + angular rates)
- `quats` – quaternion history `[w, x, y, z]`
- `omegas` – angular velocity history in the body frame
- `qRef`, `wRef` – reference attitude and rate histories
- `attitudeError` – attitude error vector in the body frame
- `rateError` – angular rate error in the body frame

The module `python/attitude_plotting.py` provides Plotly utilities for:

- Quaternion time histories
- Euler angles time histories (roll, pitch, yaw)
- 3D attitude animation
- Rotational kinetic energy vs time
- Attitude and rate error vs time
