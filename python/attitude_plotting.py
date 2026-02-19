# python/attitude_plotting.py

import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R


# ------------------------------------------------
# Common constants
# ------------------------------------------------
_FONT = "Courier New, monospace"
_BG = "rgba(10,10,15,1)"
_PAPER_BG = "rgba(0,0,0,1)"
_GRID = "rgba(255,255,255,0.1)"

# Consistent axis colors: x=red, y=green, z=blue
_XYZ_COLORS = ["#FF5555", "#55FF55", "#5599FF"]

# Quaternion colors: w=cyan, x=red, y=green, z=blue
_QUAT_COLORS = ["#00FFFF", "#FF5555", "#55FF55", "#5599FF"]

# Single-trace accent color
_ACCENT = "#00FFCC"


# ------------------------------------------------
# Common styling helper
# ------------------------------------------------
def _apply_style(fig, title, x_label, y_label):
    fig.update_layout(
        title=dict(
            text=title,
            font=dict(family=_FONT, size=22, color="#FFFFFF"),
            x=0.5,
        ),
        xaxis=dict(
            title=dict(text=x_label, font=dict(family=_FONT, size=14, color="#FFFFFF")),
            gridcolor=_GRID,
            zeroline=False,
        ),
        yaxis=dict(
            title=dict(text=y_label, font=dict(family=_FONT, size=14, color="#FFFFFF")),
            gridcolor=_GRID,
            zeroline=False,
        ),
        template="plotly_dark",
        font=dict(family=_FONT, size=13, color="#FFFFFF"),
        legend=dict(
            x=1.02, y=1.0, xanchor="left", yanchor="top",
            bgcolor="rgba(0,0,0,0)", borderwidth=0,
            font=dict(family=_FONT, size=12, color="#FFFFFF"),
        ),
        margin=dict(l=60, r=140, t=80, b=60),
        height=500,
        plot_bgcolor=_BG,
        paper_bgcolor=_PAPER_BG,
    )
    return fig


def _add_xyz_traces(fig, t, data, labels, colors=_XYZ_COLORS):
    for i in range(3):
        fig.add_trace(go.Scatter(
            x=t, y=data[:, i], mode="lines",
            name=labels[i], line=dict(width=2.5, color=colors[i]),
        ))
    return fig


def _add_single_trace(fig, t, y, name, color=_ACCENT):
    fig.add_trace(go.Scatter(
        x=t, y=y, mode="lines",
        name=name, line=dict(width=2.5, color=color),
    ))
    return fig


# ------------------------------------------------
# Quaternion components
# ------------------------------------------------
def plot_quaternion_components(result):
    t = np.array(result.time)
    quats = np.array(result.quats)  # (N, 4) as [w, x, y, z]

    labels = ["w", "x", "y", "z"]
    fig = go.Figure()
    for i in range(4):
        fig.add_trace(go.Scatter(
            x=t, y=quats[:, i], mode="lines",
            name=labels[i], line=dict(width=2.5, color=_QUAT_COLORS[i]),
        ))

    _apply_style(fig, "Quaternion Components", "Time [s]", "Component Value")
    fig.show()


# ------------------------------------------------
# Euler angles
# ------------------------------------------------
def plot_euler_angles(result):
    t = np.array(result.time)
    quats = np.array(result.quats)

    quats_xyzw = np.stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]], axis=1)
    euler = R.from_quat(quats_xyzw).as_euler("xyz", degrees=True)

    fig = go.Figure()
    _add_xyz_traces(fig, t, euler, ["Roll", "Pitch", "Yaw"])
    _apply_style(fig, "Euler Angles", "Time [s]", "Angle [deg]")
    fig.show()


# ------------------------------------------------
# Angular velocity
# ------------------------------------------------
def plot_angular_velocity(result):
    t = np.array(result.time)
    omegas = np.array(result.omegas)

    fig = go.Figure()
    _add_xyz_traces(fig, t, omegas, ["wx", "wy", "wz"])
    _apply_style(fig, "Angular Velocity", "Time [s]", "Angular Velocity [rad/s]")
    fig.show()


# ------------------------------------------------
# Rotational kinetic energy
# ------------------------------------------------
def plot_rotational_kinetic_energy(result, inertia_body):
    t = np.array(result.time)
    omegas = np.array(result.omegas)
    J = np.array(inertia_body, dtype=float).reshape(3, 3)

    J_omega = omegas @ J.T
    T = 0.5 * np.sum(omegas * J_omega, axis=1)

    fig = go.Figure()
    _add_single_trace(fig, t, T, "Rotational KE")
    _apply_style(fig, "Rotational Kinetic Energy", "Time [s]", "Energy [J]")
    fig.show()


# ------------------------------------------------
# Attitude error components
# ------------------------------------------------
def plot_attitude_error_components(result):
    t = np.array(result.time)
    e_att = np.array(result.attitudeError)

    fig = go.Figure()
    _add_xyz_traces(fig, t, e_att, ["e_x", "e_y", "e_z"])
    _apply_style(fig, "Attitude Error Components", "Time [s]", "Error [rad]")
    fig.show()


# ------------------------------------------------
# Attitude error norm
# ------------------------------------------------
def plot_attitude_error_norm(result):
    t = np.array(result.time)
    e_att = np.array(result.attitudeError)
    e_norm_deg = np.rad2deg(np.linalg.norm(e_att, axis=1))

    fig = go.Figure()
    _add_single_trace(fig, t, e_norm_deg, "||e_att||")
    _apply_style(fig, "Attitude Error Norm", "Time [s]", "||e_att|| [deg]")
    fig.show()


# ------------------------------------------------
# Rate error components
# ------------------------------------------------
def plot_rate_error_components(result):
    t = np.array(result.time)
    e_rate = np.array(result.rateError)

    fig = go.Figure()
    _add_xyz_traces(fig, t, e_rate, ["e_wx", "e_wy", "e_wz"])
    _apply_style(fig, "Rate Error Components", "Time [s]", "Rate Error [rad/s]")
    fig.show()


# ------------------------------------------------
# Rate error norm
# ------------------------------------------------
def plot_rate_error_norm(result):
    t = np.array(result.time)
    e_rate = np.array(result.rateError)
    e_norm = np.linalg.norm(e_rate, axis=1)

    fig = go.Figure()
    _add_single_trace(fig, t, e_norm, "||e_w||")
    _apply_style(fig, "Rate Error Norm", "Time [s]", "||e_w|| [rad/s]")
    fig.show()


# ------------------------------------------------
# Commanded vs. applied torque
# ------------------------------------------------
def plot_torque(result):
    # Torque vectors are size N; time is size N+1
    t = np.array(result.time[:-1])
    tau_cmd = np.array(result.commandedTorque)
    tau_app = np.array(result.appliedTorque)

    axis_labels = ["x", "y", "z"]
    fig = go.Figure()
    for i in range(3):
        fig.add_trace(go.Scatter(
            x=t, y=tau_cmd[:, i], mode="lines",
            name=f"cmd {axis_labels[i]}",
            line=dict(width=2, color=_XYZ_COLORS[i], dash="dash"),
        ))
        fig.add_trace(go.Scatter(
            x=t, y=tau_app[:, i], mode="lines",
            name=f"app {axis_labels[i]}",
            line=dict(width=2.5, color=_XYZ_COLORS[i]),
        ))

    _apply_style(fig, "Commanded vs. Applied Torque", "Time [s]", "Torque [N·m]")
    fig.show()
