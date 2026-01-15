import starSense
from lqr_utils import build_lqr_gain
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot_rotational_kinetic_energy,
    plot_attitude_error_components,
    plot_attitude_error_norm,
    plot_rate_error_components,
    plot_rate_error_norm,
)

# set up simulation parameters
params = starSense.AttitudeSimParams()
params.dt = 0.001
params.numSteps = 50000

# spacecraft parameters
params.q0 = [1.0, 0.05, -0.03, 0.02]  # small-ish attitude error
params.w0 = [0.1, -0.05, 0.02]
params.inertiaBody = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0], 
]

# constant reference profile
params.referenceType = 'fixed'
params.qRef = [1.0, 0.0, 0.0, 0.0]
params.wRef = [0.0, 0.0, 0.0]

# control parameters
params.controllerType = 'lqr'
q_wts = [0.2, 0.2, 0.2]   # attitude
w_wts = [0.9, 0.9, 0.9]   # rate
r_wts = [0.5, 0.5, 0.5]   # control effort

K_lqr = build_lqr_gain(params.inertiaBody, q_wts, w_wts, r_wts)  # 3x6
params.kLqr = K_lqr.tolist()
params.controlRateHz = 1

# run simulation
out = starSense.run_simulation(params)

# rotation plots
plot_quaternion_components(out)
plot_euler_angles(out)

# energy plots
plot_rotational_kinetic_energy(out, params.inertiaBody)

# control error  plots
plot_attitude_error_components(out)
plot_attitude_error_norm(out)
plot_rate_error_components(out)
plot_rate_error_norm(out)
