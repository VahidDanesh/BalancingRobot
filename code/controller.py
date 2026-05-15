"""Phase II — Controller Design.

Two controllers are designed:
    1. Stabilizing LQR  : u = -K @ x
       State order: [x, theta, x_dot, theta_dot]
       Q = diag(1, 10, 0, 0), R = 0.1
       Weights: tilt penalty 10× position; velocities unpenalized.

    2. Tracking LQR     : u = -K_aug @ [x; integral(x - x_ref)]
       Augmented (5-state) system for zero steady-state error on position.

    3. (Optional) Feedback Linearization + LQR : u = fbl_control(x, K_fl)
       Input-output linearization for output y = theta (relative degree 2).
       Exact cancellation of nonlinear terms in theta_ddot, then virtual
       LQR input v = -K_fl @ x drives the linearized theta subsystem.

Both designs verified: all closed-loop poles strictly in the LHP.

Exports
-------
design_lqr(A, B, Q, R)           -> K, cl_poles
design_tracking(A, B, ...)        -> K_aug, A_aug, B_aug, aug_poles
fbl_control(x_state, K_fl, params) -> u  (float)
"""

import numpy as np
import control as ct


# ---------------------------------------------------------------------------
# Default weight matrices
# State order: [x, theta, x_dot, theta_dot]  →  indices [0, 1, 2, 3]
# ---------------------------------------------------------------------------
Q_DEFAULT     = np.diag([1.0, 10.0, 0.0, 0.0])   # penalize x (idx 0) and theta (idx 1)
R_DEFAULT     = np.array([[0.1]])
Q_AUG_DEFAULT = np.diag([1.0, 10.0, 0.0, 0.0, 5.0])  # add integral-error weight


def design_lqr(A, B, Q=None, R=None):
    """Design LQR state-feedback stabilizing controller.

    Solves the algebraic Riccati equation and returns the optimal gain K
    such that u = -K @ x minimizes J = ∫(x'Qx + u'Ru) dt.

    Parameters
    ----------
    A, B : ndarray — linearized system matrices (4×4, 4×1)
    Q    : ndarray or None — state cost (4×4, PSD); uses default if None
    R    : ndarray or None — input cost (1×1, PD);  uses default if None

    Returns
    -------
    K        : ndarray (1×4) — LQR gain row vector
    cl_poles : ndarray (4,)  — eigenvalues of (A - B @ K)
    """
    if Q is None:
        Q = Q_DEFAULT
    if R is None:
        R = R_DEFAULT

    K, _, _ = ct.lqr(A, B, Q, R)
    cl_poles = np.linalg.eigvals(A - B @ K)
    assert np.all(np.real(cl_poles) < 0), \
        "LQR: closed-loop has at least one unstable pole."
    return K, cl_poles


def design_tracking(A, B, Q_aug=None, R_aug=None):
    """Design augmented LQR for zero-steady-state-error position tracking.

    Augmented state:
        x_aug = [x, theta, x_dot, theta_dot, xi]^T  in R^5
    where xi = integral(x - x_ref) dt.

    Augmented dynamics:
        A_aug = [[A,      0],      B_aug = [[B],
                 [e1^T,   0]]               [0]]
    where e1^T = [1, 0, 0, 0] selects x (index 0).

    Control law:  u = -K_aug @ x_aug

    Parameters
    ----------
    A, B    : ndarray — nominal 4×4 and 4×1 matrices
    Q_aug   : ndarray or None — 5×5 state cost; uses default if None
    R_aug   : ndarray or None — 1×1 input cost; uses default if None

    Returns
    -------
    K_aug     : ndarray (1×5)  — augmented gain
    A_aug     : ndarray (5×5)  — augmented state matrix
    B_aug     : ndarray (5×1)  — augmented input matrix
    aug_poles : ndarray (5,)   — eigenvalues of (A_aug - B_aug @ K_aug)
    """
    if Q_aug is None:
        Q_aug = Q_AUG_DEFAULT
    if R_aug is None:
        R_aug = R_DEFAULT

    n = A.shape[0]
    e1 = np.zeros((1, n))
    e1[0, 0] = 1.0  # selects x (index 0)

    A_aug = np.block([[A,   np.zeros((n, 1))],
                      [e1,  np.zeros((1, 1))]])
    B_aug = np.block([[B],
                      [np.zeros((1, 1))]])

    K_aug, _, _ = ct.lqr(A_aug, B_aug, Q_aug, R_aug)
    aug_poles = np.linalg.eigvals(A_aug - B_aug @ K_aug)
    assert np.all(np.real(aug_poles) < 0), \
        "Tracking LQR: augmented closed-loop has at least one unstable pole."
    return K_aug, A_aug, B_aug, aug_poles


# ---------------------------------------------------------------------------
# Feedback Linearization controller (optional Phase II)
# ---------------------------------------------------------------------------

def fbl_control(x_state, K_fl, params=None):
    """Feedback-linearization + virtual LQR control for the TWSBR.

    Chooses output y = theta (tilt angle), which has relative degree 2.
    Exactly cancels the nonlinear terms in theta_ddot so that
        theta_ddot = v  (double-integrator subsystem)
    where the virtual input v = -K_fl @ x_state is chosen by the LQR law.

    FL inversion formula (state order [x, theta, x_dot, theta_dot]):
        theta_ddot = [(M+m)*m*g*L*sin(theta)
                      - m*L*cos(theta)*(u - b*xdot + m*L*thdot^2*sin(theta))] / Delta
        Setting theta_ddot = v and solving for u:
            u = [(M+m)*m*g*L*sin(theta) - v*Delta] / (m*L*cos(theta))
                + b*xdot - m*L*thdot^2*sin(theta)

    Parameters
    ----------
    x_state : array (4,) — current state estimate [x, theta, x_dot, theta_dot]
    K_fl    : ndarray (1×4) — LQR outer-loop gain (same as stabilizing K)
    params  : dict or None — override physical parameters (robustness sweeps)

    Returns
    -------
    u : float — control input (N)

    Notes
    -----
    Singularity at theta = ±pi/2 (cos theta = 0).  A guard returns the
    linear LQR value when |cos(theta)| < 1e-3 to avoid division-by-zero.
    """
    from parameters import M as M0, m as m0, L as L0, I as I0, b as b0, g as g0

    _M = params['M'] if params and 'M' in params else M0
    _m = params['m'] if params and 'm' in params else m0
    _L = params['L'] if params and 'L' in params else L0
    _I = params['I'] if params and 'I' in params else I0
    _b = params['b'] if params and 'b' in params else b0
    _g = params['g'] if params and 'g' in params else g0

    x_arr = np.asarray(x_state).flatten()
    _, theta, xdot, thdot = x_arr

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    Delta = (_M + _m) * (_I + _m * _L**2) - (_m * _L * cos_t)**2

    # Virtual LQR input
    v = float(np.asarray(-K_fl @ x_arr).flat[0])

    # Singularity guard — near theta = ±90°, fall back to linear control
    if abs(cos_t) < 1e-3:
        return v

    # FL inversion: u such that theta_ddot = v
    u = ((_M + _m) * _m * _g * _L * sin_t - v * Delta) / (_m * _L * cos_t) \
        + _b * xdot - _m * _L * thdot**2 * sin_t
    return float(u)
