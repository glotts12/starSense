# python/attitude_plotting.py

import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R


# ---------------------------- #
#   Common styling helper
# ---------------------------- #
def _apply_common_style(fig, title_text, x_title, y_title=None):
    """Apply unified dark neon aesthetic for 2D plots."""
    fig.update_layout(
        title=dict(
            text=title_text,
            font=dict(family="Courier New, monospace", size=24, color="#FFFFFF"),
            x=0.5,
        ),
        xaxis=dict(
            title=dict(
                text=x_title,
                font=dict(family="Courier New, monospace", size=16, color="#FFFFFF"),
            ),
            gridcolor="rgba(255,255,255,0.1)",
            zeroline=False,
        ),
        template="plotly_dark",
        font=dict(family="Courier New, monospace", size=14, color="#FFFFFF"),
        legend=dict(
            x=1.02,
            y=1.0,
            xanchor="left",
            yanchor="top",
            bgcolor="rgba(0,0,0,0)",
            borderwidth=0,
            font=dict(family="Courier New, monospace", size=12, color="#FFFFFF"),
        ),
        margin=dict(l=60, r=140, t=80, b=60),
        height=550,
        plot_bgcolor="rgba(10,10,15,1)",
        paper_bgcolor="rgba(0,0,0,1)",
    )

    # Only set y-axis title for 2D plots
    if y_title is not None and "yaxis" in fig.layout:
        fig.update_yaxes(
            title=dict(
                text=y_title,
                font=dict(family="Courier New, monospace", size=16, color="#FFFFFF"),
            ),
            gridcolor="rgba(255,255,255,0.1)",
            zeroline=False,
        )

    return fig


# ---------------------------- #
#   1. Quaternion components
# ---------------------------- #
def plot_quaternion_components(result):
    """
    Plot quaternion components vs time with neon styling.

    Parameters
    ----------
    result : BallisticSimOutput
        Output from starSense.run_ballistic_simulation
        Must have result.time and result.quats (list of [w,x,y,z]).
    """
    t = np.array(result.time)
    quats = np.array(result.quats)  # shape (N, 4) as [w, x, y, z]

    w = quats[:, 0]
    x = quats[:, 1]
    y = quats[:, 2]
    z = quats[:, 3]

    colors = ["#00FFFF", "#00FF88", "#FF00FF", "#FF8800"]
    labels = ["w 🌀", "x ⚡", "y 💫", "z 🌌"]
    data = [w, x, y, z]

    fig = go.Figure()
    for d, label, color in zip(data, labels, colors):
        fig.add_trace(
            go.Scatter(
                x=t,
                y=d,
                mode="lines",
                name=f"<b>{label}</b>",
                line=dict(width=3, color=color),
            )
        )

    fig = _apply_common_style(
        fig,
        "🧭 Quaternion Time Evolution",
        "⏱️ Time [s]",
        "Component Value 🧮",
    )
    fig.show()


# ---------------------------- #
#   2. Euler Angle Evolution
# ---------------------------- #
def plot_euler_angles(result):
    """
    Plot roll, pitch, yaw evolution from quaternion trajectory.

    Parameters
    ----------
    result : BallisticSimOutput
        Output from starSense.run_ballistic_simulation
        Must have result.time and result.quats (list of [w,x,y,z]).
    """
    t = np.array(result.time)
    quats = np.array(result.quats)  # [w, x, y, z]

    # Convert to [x, y, z, w] for scipy
    quats_xyzw = np.stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]], axis=1)
    euler = R.from_quat(quats_xyzw).as_euler("xyz", degrees=True)

    labels = ["Roll [°] 🔴", "Pitch [°] 🟢", "Yaw [°] 🔵"]
    colors = ["#FF5555", "#55FF55", "#5599FF"]

    fig = go.Figure()
    for i in range(3):
        fig.add_trace(
            go.Scatter(
                x=t,
                y=euler[:, i],
                mode="lines",
                name=f"<b>{labels[i]}</b>",
                line=dict(width=3, color=colors[i]),
            )
        )

    fig = _apply_common_style(
        fig,
        "🪩 Euler Angle Evolution",
        "⏱️ Time [s]",
        "Angle [deg] ⚙️",
    )
    fig.show()


# ---------------------------- #
#   3. 3D Quaternion Animation
# ---------------------------- #
def plot3d_orientation_animation(result):
    """
    Animated 3D visualization of body axes orientation vs time.

    Parameters
    ----------
    result : BallisticSimOutput
        Output from starSense.run_ballistic_simulation
        Must have result.time and result.quats (list of [w,x,y,z]).
    """
    t = np.array(result.time)
    quats = np.array(result.quats)  # [w, x, y, z]

    quats_xyzw = np.stack([quats[:, 1], quats[:, 2], quats[:, 3], quats[:, 0]], axis=1)
    rotations = R.from_quat(quats_xyzw)
    rot_mats = rotations.as_matrix()  # shape (N, 3, 3)

    axis_length = 1.0
    body_x = np.array([axis_length, 0.0, 0.0])
    body_y = np.array([0.0, axis_length, 0.0])
    body_z = np.array([0.0, 0.0, axis_length])

    frames = []
    for i in range(len(rot_mats)):
        R_i = rot_mats[i]
        origin = np.array([0.0, 0.0, 0.0])
        x_vec = R_i @ body_x
        y_vec = R_i @ body_y
        z_vec = R_i @ body_z

        frames.append(
            go.Frame(
                data=[
                    go.Scatter3d(
                        x=[origin[0], x_vec[0]],
                        y=[origin[1], x_vec[1]],
                        z=[origin[2], x_vec[2]],
                        mode="lines+markers",
                        line=dict(color="#FF5555", width=6),
                        name="X-axis",
                        showlegend=False,
                    ),
                    go.Scatter3d(
                        x=[origin[0], y_vec[0]],
                        y=[origin[1], y_vec[1]],
                        z=[origin[2], y_vec[2]],
                        mode="lines+markers",
                        line=dict(color="#55FF55", width=6),
                        name="Y-axis",
                        showlegend=False,
                    ),
                    go.Scatter3d(
                        x=[origin[0], z_vec[0]],
                        y=[origin[1], z_vec[1]],
                        z=[origin[2], z_vec[2]],
                        mode="lines+markers",
                        line=dict(color="#5599FF", width=6),
                        name="Z-axis",
                        showlegend=False,
                    ),
                ],
                name=f"t={t[i]:.2f}s",
            )
        )

    # Initial frame
    fig = go.Figure(data=frames[0].data)

    fig.update_layout(
        title=dict(
            text="🌀 3D Quaternion Orientation Animation",
            font=dict(family="Courier New, monospace", size=24, color="#00FFCC"),
            x=0.5,
        ),
        margin=dict(l=0, r=0, b=0, t=60),
        scene=dict(
            xaxis=dict(title="X", range=[-1.5, 1.5], gridcolor="rgba(255,255,255,0.1)"),
            yaxis=dict(title="Y", range=[-1.5, 1.5], gridcolor="rgba(255,255,255,0.1)"),
            zaxis=dict(title="Z", range=[-1.5, 1.5], gridcolor="rgba(255,255,255,0.1)"),
            aspectmode="cube",
            bgcolor="rgba(10,10,15,1)",
        ),
        template="plotly_dark",
        font=dict(family="Courier New, monospace", size=14, color="#FFFFFF"),
        paper_bgcolor="rgba(0,0,0,1)",
        updatemenus=[
            dict(
                type="buttons",
                buttons=[
                    dict(
                        label="▶️ Play",
                        method="animate",
                        args=[
                            None,
                            {
                                "frame": {"duration": 50, "redraw": True},
                                "fromcurrent": True,
                                "transition": {"duration": 0},
                            },
                        ],
                    ),
                    dict(
                        label="⏸️ Pause",
                        method="animate",
                        args=[
                            [None],
                            {
                                "mode": "immediate",
                                "frame": {"duration": 0, "redraw": False},
                                "transition": {"duration": 0},
                            },
                        ],
                    ),
                ],
                direction="left",
                pad={"r": 10, "t": 10},
                showactive=False,
                x=0.1,
                xanchor="left",
                y=1.1,
                yanchor="top",
            )
        ],
        sliders=[
            dict(
                steps=[
                    dict(
                        method="animate",
                        args=[
                            [f.name],
                            {
                                "mode": "immediate",
                                "frame": {"duration": 0, "redraw": True},
                                "transition": {"duration": 0},
                            },
                        ],
                        label=f"t={t[i]:.2f}s",
                    )
                    for i, f in enumerate(frames)
                ],
                active=0,
                x=0.1,
                y=0.0,
                xanchor="left",
                yanchor="top",
                len=0.8,
                font=dict(family="Courier New, monospace", size=12, color="#FFFFFF"),
                bgcolor="rgba(0,0,0,0)",
            )
        ],
    )

    fig.frames = frames
    fig.show()
