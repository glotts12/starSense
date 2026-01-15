# python/lqr_utils.py
import numpy as np

def build_lqr_gain(inertia_body, q_weights, w_weights, r_weights):
    """
    Build a continuous-time LQR gain K for the attitude error state:
        x = [phi_x, phi_y, phi_z, e_wx, e_wy, e_wz]^T

    Dynamics (linearized):
        phi_dot = e_w
        e_w_dot = J^{-1} * tau     (assuming w_ref = 0 and ignoring cross-terms)

    So:
        A = [[0,  I],
             [0,  0]]  (6x6)
        B = [[0],
             [J^{-1}]]  (6x3)

    Cost:
        J = ∫ (x^T Q x + u^T R u) dt

    Parameters
    ----------
    inertia_body : array-like, shape (3, 3)
        Inertia matrix J in body frame.
    q_weights : array-like, length 3
        State weights on attitude error components (phi_x, phi_y, phi_z).
    w_weights : array-like, length 3
        State weights on rate error components (e_wx, e_wy, e_wz).
    r_weights : array-like, length 3
        Control effort weights on torque components (tau_x, tau_y, tau_z).

    Returns
    -------
    K : np.ndarray, shape (3, 6)
        LQR state feedback gain such that:
            u = -K x
        where x = [phi; e_w].
    """
    try:
        from control import lqr
    except ImportError as e:
        raise ImportError(
            "python-control is required for build_lqr_gain. "
            "Install with `pip install control`."
        ) from e

    J = np.array(inertia_body, dtype=float)
    if J.shape != (3, 3):
        raise ValueError(f"inertia_body must be 3x3, got shape {J.shape}")

    # Check SPD-ish and invertible
    if np.linalg.det(J) == 0.0:
        raise ValueError("Inertia matrix J is singular; cannot build LQR gain.")

    J_inv = np.linalg.inv(J)

    # State: x = [phi(3); e_w(3)]  -> size 6
    A = np.zeros((6, 6))
    A[0:3, 3:6] = np.eye(3)  # phi_dot = e_w

    B = np.zeros((6, 3))
    B[3:6, :] = J_inv        # e_w_dot = J^{-1} tau

    q_weights = np.asarray(q_weights, dtype=float).reshape(3)
    w_weights = np.asarray(w_weights, dtype=float).reshape(3)
    r_weights = np.asarray(r_weights, dtype=float).reshape(3)

    Q_diag = np.concatenate([q_weights, w_weights])  # length 6
    Q = np.diag(Q_diag)
    R = np.diag(r_weights)

    K, S, E = lqr(A, B, Q, R)  # K: 3x6, u = -K x by convention

    # Ensure it's a real-valued ndarray
    K = np.asarray(K, dtype=float)
    return K
