import starSense
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot3d_orientation_animation,
)

# set up simulation parameters
params = starSense.AttitudeSimParams()
params.dt = 0.1
params.numSteps = 5000
params.q0 = [1.0, 0.0, 0.0, 0.0]
params.inertiaBody = [
    [2.0, 0.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.0, 0.0, 1.0],
]
params.w0 = [0.02, 0.0, 0.1]

# run simulation
out = starSense.run_simulation(params)

plot_quaternion_components(out)
plot_euler_angles(out)
plot3d_orientation_animation(out)
