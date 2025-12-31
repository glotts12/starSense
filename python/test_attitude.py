import starSense
from attitude_plotting import (
    plot_quaternion_components,
    plot_euler_angles,
    plot3d_orientation_animation,
)

params = starSense.BallisticSimParams()
params.q0 = [1.0, 0.0, 0.0, 0.0]
params.w0 = [0.20, 0.9, 0.1]
params.dt = 0.1
params.numSteps = 1000
params.integratorType = "rk4"

out = starSense.run_ballistic_simulation(params)

plot_quaternion_components(out)
plot_euler_angles(out)
plot3d_orientation_animation(out)
