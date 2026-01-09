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
params.numSteps = 5000
params.q0 = [1.0, 0.0, 0.0, 0.0]
params.inertiaBody = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0], 
]
params.w0 = [1.0, 0.0, 0.0] 

# constant reference profile
params.referenceType = 'fixed'
params.wRef = [1.0, 0.0, 0.0] 
params.qRef = [1.0, 0.0, 0.0, 0.0]

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
