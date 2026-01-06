import starSense
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot3d_orientation_animation,
    plot_rotational_kinetic_energy,
)

# set up simulation parameters
params = starSense.AttitudeSimParams()
params.dt = 0.1
params.numSteps = 500
params.q0 = [1.0, 0.0, 0.0, 0.0]
params.inertiaBody = [
    [1.0, 0.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.0, 0.0, 3.0], 
]
params.w0 = [1.0, 0.5, 0.1] 

# run simulation
out = starSense.run_simulation(params)

plot_quaternion_components(out)
plot_euler_angles(out)
plot_rotational_kinetic_energy(out, params.inertiaBody)
plot3d_orientation_animation(out)
