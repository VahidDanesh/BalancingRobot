"""Phase II — Observer Design.

State order (CLAUDE.md [q; q_dot] convention):
    x_state = [x, theta, x_dot, theta_dot]   indices [0, 1, 2, 3]

Two observers are designed:

    1. Full-order Luenberger (4th order)
       Estimates all states from y = [x, theta].
       Observer poles placed ≥ 3× faster than slowest controller pole.
       Gain L ∈ R^{4×2} via pole placement on (A - L @ C).

    2. Reduced-order Luenberger (2nd order)
       x_a = [x, theta]       — directly measured  (indices 0, 1)
       x_b = [x_dot, theta_dot] — estimated        (indices 2, 3)
       Internal state: xi = x_b_hat - L_r @ x_a
       Stability condition: eig(A_bb - L_r @ A_ab) in LHP.
       Since A_ab = I_2 (top-right block of A), condition simplifies to
       eig(A_bb - L_r) in LHP.

Both designs verified: all observer poles strictly in the LHP.

Exports
-------
design_full_observer(A, C, obs_poles)          -> L, obs_eigs
design_reduced_observer(A, obs_poles_r)        -> L_r, red_eigs, partition
observer_pole_targets(cl_poles, factor=4.0)    -> obs_poles (4,)
"""

import numpy as np
import control as ct


def observer_pole_targets(cl_poles, factor=4.0, spread=0.1):
    """Compute observer pole targets as a multiple of the controller poles.

    Observer poles are placed at factor × the real part of each controller
    pole, ensuring the observer is faster than the closed-loop dynamics.
    A small spread is added so no two poles are repeated (control.place
    cannot handle repeated poles).

    Parameters
    ----------
    cl_poles : array-like (n,) — closed-loop controller poles
    factor   : float — speed multiplier (≥ 3 per Hespanha rule)
    spread   : float — fractional spread between poles to avoid repetition

    Returns
    -------
    obs_poles : ndarray (n,) — target observer poles (all real, all negative)
    """
    n = len(cl_poles)
    base = factor * np.max(np.abs(np.real(cl_poles)))
    obs_poles = np.array([-(base * (1.0 + spread * i)) for i in range(n)])
    return obs_poles


def design_full_observer(A, C, obs_poles):
    """Design full-order Luenberger observer by pole placement.

    Finds L ∈ R^{4×2} such that eig(A - L @ C) equals obs_poles.

    Observer dynamics:
        x_hat_dot = A @ x_hat + B @ u + L @ (y - C @ x_hat)

    Error dynamics:
        e_dot = (A - L @ C) @ e

    Parameters
    ----------
    A         : ndarray (4×4)
    C         : ndarray (2×4)
    obs_poles : array-like (4,) — desired poles (all strictly in LHP)

    Returns
    -------
    L        : ndarray (4×2) — observer gain
    obs_eigs : ndarray (4,) — achieved eigenvalues of (A - L @ C)
    """
    L = ct.place(A.T, C.T, obs_poles).T
    obs_eigs = np.linalg.eigvals(A - L @ C)
    assert np.all(np.real(obs_eigs) < 0), \
        "Full observer: (A - L@C) has at least one unstable eigenvalue."
    return L, obs_eigs


def design_reduced_observer(A, obs_poles_r):
    """Design 2nd-order reduced-order observer for velocity states.

    State partition (state order: [x, theta, x_dot, theta_dot]):
        x_a = [x, theta]           -> indices [0, 1]  (measured)
        x_b = [x_dot, theta_dot]   -> indices [2, 3]  (estimated)

    Sub-block extraction:
        A_aa = A[[0,1], :][:, [0,1]],  A_ab = A[[0,1], :][:, [2,3]]
        A_ba = A[[2,3], :][:, [0,1]],  A_bb = A[[2,3], :][:, [2,3]]

    Note: A_ab = I_2 (the top-right block of A is the identity),
    so stability condition A_bb - L_r @ A_ab = A_bb - L_r.

    Internal observer state: xi = x_b_hat - L_r @ x_a
    Dynamics of xi:
        xi_dot = F @ xi + G @ x_a + H @ u
    where:
        F = A_bb - L_r @ A_ab
        G = F @ L_r + A_ba - L_r @ A_aa
        H = B_b - L_r @ B_a   (computed in simulate.py using B)

    Reconstruction:
        x_b_hat = xi + L_r @ x_a

    Parameters
    ----------
    A           : ndarray (4×4)
    obs_poles_r : array-like (2,) — desired reduced observer poles

    Returns
    -------
    L_r      : ndarray (2×2) — reduced observer gain
    red_eigs : ndarray (2,) — eigenvalues of (A_bb - L_r @ A_ab)
    partition : dict with sub-block matrices and index sets
    """
    idx_a = [0, 1]   # measured  : [x, theta]
    idx_b = [2, 3]   # unmeasured: [x_dot, theta_dot]

    A_aa = A[np.ix_(idx_a, idx_a)]   # (2×2)
    A_ab = A[np.ix_(idx_a, idx_b)]   # (2×2)  = I_2 for this system
    A_ba = A[np.ix_(idx_b, idx_a)]   # (2×2)
    A_bb = A[np.ix_(idx_b, idx_b)]   # (2×2)

    L_r = ct.place(A_bb.T, A_ab.T, obs_poles_r).T
    red_eigs = np.linalg.eigvals(A_bb - L_r @ A_ab)
    assert np.all(np.real(red_eigs) < 0), \
        "Reduced observer: (A_bb - L_r @ A_ab) has at least one unstable eigenvalue."

    F = A_bb - L_r @ A_ab
    G = F @ L_r + A_ba - L_r @ A_aa

    partition = {
        'idx_a': idx_a, 'idx_b': idx_b,
        'A_aa': A_aa, 'A_ab': A_ab, 'A_ba': A_ba, 'A_bb': A_bb,
        'F': F, 'G': G,
    }
    return L_r, red_eigs, partition
