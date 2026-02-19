import starSense
from lqr_utils import build_lqr_gain
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot_angular_velocity,
    plot_rotational_kinetic_energy,
    plot_attitude_error_components,
    plot_attitude_error_norm,
    plot_rate_error_components,
    plot_rate_error_norm,
    plot_torque,
)

# set up simulation parameters
params = starSense.AttitudeSimParams()
params.dt = 0.01
params.numSteps = 45000

# spacecraft parameters
params.q0 = [1.0, 0, 0, 0]  # small-ish attitude error
params.w0 = [1.3, 2, 3.1]
params.inertiaBody = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0], 
]

# spinnig reference profile
params.referenceType = 'spinning'
params.wRef = [1.0, 0.0, 0.0]

# control parameters
params.controllerType = 'pd'
params.kpAtt  = [0.25, 0.25, 0.25]
params.kdRate = [0.7,  0.7,  0.7]
params.controlRateHz = 1

# actuator configuration
params.actuatorType = 'reactionWheel'
params.wheelAxes = [
    [1.0, 0.0, 0.0],  # wheel 1 spin axis in body frame
    [0.0, 1.0, 0.0],  # wheel 2
    [0.0, 0.0, 1.0],  # wheel 3
]

# wheel properties (per wheel)
params.wheelInertias = [0.01, 0.01, 0.01]  # kg·m² (spin axis MOI)

# saturation limits
params.maxWheelTorque = [0.1, 0.1, 0.1]      # N·m
params.maxWheelSpeed  = [6000, 6000, 6000]   # RPM

# initial wheel speeds
params.wheelSpeeds0 = [0.0, 0.0, 0.0]  # RPM

# run simulation
out = starSense.run_simulation(params)

# rotation plots
plot_quaternion_components(out)
plot_euler_angles(out)

# angular velocity
plot_angular_velocity(out)

# energy plot
plot_rotational_kinetic_energy(out, params.inertiaBody)

# control error plots
plot_attitude_error_components(out)
plot_attitude_error_norm(out)
plot_rate_error_components(out)
plot_rate_error_norm(out)
plot_torque(out)
