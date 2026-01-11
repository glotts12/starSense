import starSense
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot3d_orientation_animation,
    plot_rotational_kinetic_energy,
    plot_attitude_error_components,
    plot_attitude_error_norm,
    plot_rate_error_components,
    plot_rate_error_norm,
)

# set up simulation parameters
params = starSense.AttitudeSimParams()
params.dt = 0.01
params.numSteps = 3000

# spacecraft parameters
params.q0 = [1.0, 0.2, 2.0, 5.0]
params.w0 = [0.8, 1.3, 2.1] 
params.inertiaBody = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0], 
]

# constant reference profile
params.referenceType = 'fixed'
params.wRef = [0.0, 0.0, 0.0] 
params.qRef = [1.0, 0.0, 0.0, 0.0]

# control parameters
params.controllerType = 'pd'
params.kpAtt  = [0.25, 0.25, 0.25]
params.kdRate = [0.7,  0.7,  0.7]
params.controlRateHz = 1

# run simulation
out = starSense.run_simulation(params)

# rotation plots
plot_quaternion_components(out)
plot_euler_angles(out)
plot3d_orientation_animation(out)

# energy plots
plot_rotational_kinetic_energy(out, params.inertiaBody)

# control error  plots
plot_attitude_error_components(out)
plot_attitude_error_norm(out)
plot_rate_error_components(out)
plot_rate_error_norm(out)
