"""Phase II & III — Closed-Loop Simulation and Plotting.

State order (CLAUDE.md [q; q_dot] convention):
    x_state = [x, theta, x_dot, theta_dot]   indices [0, 1, 2, 3]

Co-simulation structure
-----------------------
Plant and observer are integrated in a single solve_ivp call to share
the same adaptive time grid (no artificial delays between them).

Simulation functions
--------------------
simulate_stabilization(x0, t_span, K, L, A, B, C, f_nl, params)
    8-state ODE: [x_plant(4), x_obs(4)]
    Control law: u = -K @ x_hat

simulate_tracking(t_span, K_aug, L, A, B, C, f_nl, x_ref_func, x0, params)
    9-state ODE: [x_plant(4), x_obs(4), xi(1)]
    Control law: u = -K_aug @ [x_hat; xi]
    Integrator:  xi_dot = x_measured - x_ref(t)   (x is state index 0 = y[0])

simulate_reduced_obs(x0, t_span, K, L_r, partition, A, B, C, f_nl, params)
    6-state ODE: [x_plant(4), xi_obs(2)]
    Reduced observer reconstructs x_b_hat from internal state xi_obs.

simulate_fbl(x0, t_span, K_fl, L, A, B, C, f_nl, params)
    8-state ODE: [x_plant(4), x_obs(4)]
    Control law: u = fbl_control(x_hat, K_fl)  — exact FL + virtual LQR

Plot functions (save as PDF via plot_utils)
--------------------------------------------
plot_poles(ol_poles, cl_poles, obs_poles, combined_poles, save_name)
plot_stabilization(results, labels, save_name)
plot_tracking(t, x_plant, x_obs, u, x_ref_vals, save_name)
plot_observer_error(t, x_plant, x_obs, save_name)
plot_observer_comparison(t, x, x_hat_full, x_hat_red, save_name)
plot_fbl_comparison(t_lqr, xp_lqr, u_lqr, t_fbl, xp_fbl, u_fbl, save_name)
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

from plot_utils import (new_fig, save_fig, finalize,
                        C_PLANT, C_OBS, C_REF, C_CTRL, PALETTE)


# ============================================================================
# Internal RHS functions
# ============================================================================

def _rhs_stabilization(t, z, A, B, C, K, L, f_nl, params):
    """8-state RHS for plant + full-order observer (stabilization)."""
    x     = z[:4]
    x_hat = z[4:]

    u = (-K @ x_hat).item()
    y = C @ x

    dx     = f_nl(t, x, u, params)
    dx_hat = A @ x_hat + B.flatten() * u + L @ (y - C @ x_hat)

    return np.concatenate([dx, dx_hat])


def _rhs_tracking(t, z, A, B, C, K_aug, L, f_nl, x_ref_func, params):
    """9-state RHS for plant + full-order observer + integrator (tracking)."""
    x     = z[:4]
    x_hat = z[4:8]
    xi    = z[8]

    u = (-K_aug @ np.append(x_hat, xi)).item()
    y = C @ x

    dx     = f_nl(t, x, u, params)
    dx_hat = A @ x_hat + B.flatten() * u + L @ (y - C @ x_hat)
    dxi    = y[0] - x_ref_func(t)   # x is state 0, directly measured as y[0]

    return np.concatenate([dx, dx_hat, [dxi]])


def _rhs_reduced_obs(t, z, A, B, C, K, L_r, partition, f_nl, params):
    """6-state RHS for plant + reduced-order observer (stabilization)."""
    idx_a = partition['idx_a']   # [0, 1] → [x, theta]
    idx_b = partition['idx_b']   # [2, 3] → [x_dot, theta_dot]
    F     = partition['F']
    G     = partition['G']

    x      = z[:4]
    xi_obs = z[4:]   # internal reduced observer state: xi = x_b_hat - L_r @ x_a

    x_a = x[idx_a]                    # directly measured
    x_b_hat = xi_obs + L_r @ x_a      # reconstructed velocity states

    # Full state estimate in [x, theta, x_dot, theta_dot] order
    x_hat = np.zeros(4)
    x_hat[idx_a] = x_a
    x_hat[idx_b] = x_b_hat

    u = (-K @ x_hat).item()

    B_b = B[idx_b]                    # shape (2,1)
    dx      = f_nl(t, x, u, params)
    dxi_obs = F @ xi_obs + G @ x_a + B_b.flatten() * u

    return np.concatenate([dx, dxi_obs])


# ============================================================================
# Public simulation functions
# ============================================================================

def simulate_stabilization(x0, t_span, K, L, A, B, C, f_nl,
                            params=None, n_points=2000):
    """Simulate closed-loop stabilization with full-order observer.

    Parameters
    ----------
    x0       : array (4,) — plant initial condition [x, theta, x_dot, theta_dot]
    t_span   : (t0, tf)   — simulation time interval
    K        : ndarray (1×4) — LQR gain
    L        : ndarray (4×2) — observer gain
    A, B, C  : linearized system matrices (used in observer ODE)
    f_nl     : callable — nonlinear plant dynamics f(t, x, u, params)
    params   : dict or None — perturbed parameters for robustness tests
    n_points : int — number of output time points

    Returns
    -------
    t       : ndarray (n_points,) — time vector
    x_plant : ndarray (4, n_points) — true plant states
    x_obs   : ndarray (4, n_points) — observer estimates
    u_hist  : ndarray (n_points,)   — control input history
    """
    z0 = np.concatenate([x0, np.zeros(4)])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    sol = solve_ivp(
        _rhs_stabilization, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-8, atol=1e-10,
        args=(A, B, C, K, L, f_nl, params)
    )
    t       = sol.t
    x_plant = sol.y[:4]
    x_obs   = sol.y[4:]
    u_hist  = -(K @ x_obs).flatten()

    return t, x_plant, x_obs, u_hist


def simulate_tracking(t_span, K_aug, L, A, B, C, f_nl, x_ref_func,
                       x0=None, params=None, n_points=2000):
    """Simulate closed-loop position tracking with augmented LQR.

    Parameters
    ----------
    t_span      : (t0, tf)
    K_aug       : ndarray (1×5) — augmented LQR gain [K | k_i]
    L           : ndarray (4×2) — full-order observer gain
    A, B, C     : linearized matrices
    f_nl        : nonlinear plant dynamics
    x_ref_func  : callable t -> float — reference position trajectory
    x0          : array (4,) or None — plant IC; defaults to zeros
    params      : dict or None
    n_points    : int

    Returns
    -------
    t, x_plant, x_obs, xi_hist, u_hist
    """
    if x0 is None:
        x0 = np.zeros(4)
    z0 = np.concatenate([x0, np.zeros(4), [0.0]])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    sol = solve_ivp(
        _rhs_tracking, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-8, atol=1e-10,
        args=(A, B, C, K_aug, L, f_nl, x_ref_func, params)
    )
    t       = sol.t
    x_plant = sol.y[:4]
    x_obs   = sol.y[4:8]
    xi_hist = sol.y[8]
    aug_states = np.vstack([x_obs, xi_hist])   # (5, n_points)
    u_hist = -(K_aug @ aug_states).flatten()
    return t, x_plant, x_obs, xi_hist, u_hist


def simulate_reduced_obs(x0, t_span, K, L_r, partition, A, B, C, f_nl,
                          params=None, n_points=2000):
    """Simulate stabilization with reduced-order observer.

    Parameters
    ----------
    x0        : array (4,) — plant initial condition
    t_span    : (t0, tf)
    K         : ndarray (1×4) — stabilizing LQR gain
    L_r       : ndarray (2×2) — reduced observer gain
    partition : dict from observer.design_reduced_observer
    A, B, C   : linearized matrices
    f_nl      : nonlinear plant dynamics
    params    : dict or None
    n_points  : int

    Returns
    -------
    t, x_plant, x_hat_red, u_hist
        x_hat_red : ndarray (4, n_points) — full reconstructed state
    """
    z0 = np.concatenate([x0, np.zeros(2)])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    sol = solve_ivp(
        _rhs_reduced_obs, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-8, atol=1e-10,
        args=(A, B, C, K, L_r, partition, f_nl, params)
    )
    t       = sol.t
    x_plant = sol.y[:4]
    xi_obs  = sol.y[4:]

    idx_a = partition['idx_a']   # [0, 1]
    idx_b = partition['idx_b']   # [2, 3]
    x_hat_red = np.zeros((4, len(t)))
    for i in range(len(t)):
        x_a     = x_plant[idx_a, i]
        x_b_hat = xi_obs[:, i] + L_r @ x_a
        x_hat_red[idx_a, i] = x_a
        x_hat_red[idx_b, i] = x_b_hat

    u_hist = -(K @ x_hat_red).flatten()
    return t, x_plant, x_hat_red, u_hist


# ============================================================================
# Plot functions
# ============================================================================

# State labels in [x, theta, x_dot, theta_dot] order
_STATE_LABELS = [r'$x$ (m)', r'$\theta$ (rad)',
                 r'$\dot{x}$ (m/s)', r'$\dot{\theta}$ (rad/s)']


def plot_poles(ol_poles, cl_poles, obs_poles, combined_poles,
               save_name='phase2_poles'):
    """Plot open-loop, closed-loop, and observer poles on the complex plane."""
    fig, ax = new_fig(width=5.5, height=4.0)
    ax = fig.axes[0]

    ax.axvline(0, color='k', linewidth=0.7, linestyle='--', alpha=0.5)
    ax.axhline(0, color='k', linewidth=0.7, linestyle='--', alpha=0.5)

    def _scatter(poles, marker, color, label, size=80):
        ax.scatter(np.real(poles), np.imag(poles),
                   marker=marker, c=color, s=size, linewidths=1.8,
                   zorder=5, label=label)

    _scatter(ol_poles,       'x', '#d62728', 'Open-loop')
    _scatter(cl_poles,       'o', '#1f77b4', 'Controller $(A-BK)$', size=60)
    _scatter(obs_poles,      's', '#ff7f0e', 'Observer $(A-LC)$', size=55)
    # _scatter(combined_poles, '+', "#4FD35A", 'Combined (8 poles)', size=70)

    ax.set_xlabel('Real part (rad/s)')
    ax.set_ylabel('Imaginary part (rad/s)')
    ax.set_title('Open-Loop, Closed-Loop and Observer Poles')
    ax.legend(loc='best')
    save_fig(finalize(fig), save_name)


def plot_stabilization(results, labels, save_name='phase2_stabilization'):
    """Plot state trajectories and control input for multiple initial conditions.

    Parameters
    ----------
    results : list of (t, x_plant, x_obs, u_hist) tuples
    labels  : list of str — legend labels per IC
    """
    fig, axes = new_fig(nrows=3, ncols=1, width=6.5, height=7.5, sharex=True)
    colors = PALETTE[:len(results)]

    for (t, xp, xo, u), lbl, col in zip(results, labels, colors):
        axes[0].plot(t, xp[0], color=col, label=lbl)          # x (index 0)
        axes[0].plot(t, xo[0], color=col, linestyle='--', alpha=0.6)

        axes[1].plot(t, np.degrees(xp[1]), color=col)          # theta (index 1)
        axes[1].plot(t, np.degrees(xo[1]), color=col, linestyle='--', alpha=0.6)

        axes[2].plot(t, u, color=col)

    axes[0].set_ylabel(r'Position $x$ (m)')
    axes[1].set_ylabel(r'Tilt $\theta$ (deg)')
    axes[2].set_ylabel(r'Force $u$ (N)')
    axes[2].set_xlabel('Time (s)')
    axes[2].axhline( 10, color='k', linewidth=0.7, linestyle=':', alpha=0.6)
    axes[2].axhline(-10, color='k', linewidth=0.7, linestyle=':', alpha=0.6)

    axes[0].set_title('LQR Closed-Loop Stabilization')

    from matplotlib.lines import Line2D
    legend_extras = [
        Line2D([0], [0], color='k', linestyle='-',  label='Plant (true)'),
        Line2D([0], [0], color='k', linestyle='--', alpha=0.6, label='Observer estimate'),
    ]
    axes[0].legend(handles=legend_extras + [
        Line2D([0], [0], color=c, label=l)
        for c, l in zip(colors, labels)
    ], loc='upper right', fontsize=8)

    save_fig(finalize(fig), save_name)


def plot_tracking(t, x_plant, x_obs, u_hist, x_ref_vals,
                  save_name='phase2_tracking'):
    """Plot position tracking response (step or time-varying reference).

    Parameters
    ----------
    x_ref_vals : ndarray (n_points,) — reference position at each time step
    """
    fig, axes = new_fig(nrows=3, ncols=1, width=6.5, height=7.5, sharex=True)

    axes[0].plot(t, x_plant[0],  color=C_PLANT, label=r'$x(t)$')       # x index 0
    axes[0].plot(t, x_ref_vals,  color=C_REF, linestyle='--',
                 linewidth=1.4, label=r'$x_\mathrm{ref}(t)$')
    axes[0].set_ylabel(r'Position $x$ (m)')
    axes[0].set_title('Position Tracking — Augmented LQR')
    axes[0].legend()

    axes[1].plot(t, np.degrees(x_plant[1]), color=C_PLANT,              # theta index 1
                 label=r'$\theta(t)$')
    axes[1].plot(t, np.degrees(x_obs[1]),   color=C_OBS,
                 linestyle='--', label=r'$\hat{\theta}(t)$')
    axes[1].axhline( np.degrees(0.35), color='k', linewidth=0.7,
                    linestyle=':', alpha=0.7, label=r'$\pm 20°$ limit')
    axes[1].axhline(-np.degrees(0.35), color='k', linewidth=0.7,
                    linestyle=':', alpha=0.7)
    axes[1].set_ylabel(r'Tilt $\theta$ (deg)')
    axes[1].legend()

    axes[2].plot(t, u_hist, color=C_CTRL)
    axes[2].axhline( 10, color='k', linewidth=0.7, linestyle=':', alpha=0.6,
                    label=r'$\pm u_\mathrm{max}$')
    axes[2].axhline(-10, color='k', linewidth=0.7, linestyle=':', alpha=0.6)
    axes[2].set_ylabel(r'Force $u$ (N)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()

    save_fig(finalize(fig), save_name)


def plot_observer_error(t, x_plant, x_obs, save_name='phase2_observer_error'):
    """Plot estimation error e(t) = x(t) - x_hat(t) for all four states."""
    e = x_plant - x_obs
    # Labels in [x, theta, x_dot, theta_dot] order
    ylabels = [r'$e_{x}$ (m)', r'$e_{\theta}$ (rad)',
               r'$e_{\dot{x}}$ (m/s)', r'$e_{\dot{\theta}}$ (rad/s)']

    fig, axes = new_fig(nrows=4, ncols=1, width=6.5, height=9.0, sharex=True)

    for i, ax in enumerate(axes.flatten()):
        ax.plot(t, e[i], color=C_OBS)
        ax.axhline(0, color='k', linewidth=0.6, linestyle='--', alpha=0.4)
        ax.set_ylabel(ylabels[i])

    axes[-1].set_xlabel('Time (s)')
    axes[0].set_title(
        r'Full-Order Observer Estimation Error $\mathbf{e}(t) = \mathbf{x}(t) - \hat{\mathbf{x}}(t)$')
    save_fig(finalize(fig), save_name)


def plot_observer_comparison(t, x_plant, x_hat_full, x_hat_red,
                              save_name='phase2_observer_comparison'):
    """Compare full-order and reduced-order observer estimates for velocity states."""
    fig, axes = new_fig(nrows=2, ncols=1, width=6.5, height=5.5, sharex=True)

    # x_dot is index 2 in [x, theta, x_dot, theta_dot]
    axes[0].plot(t, x_plant[2],    color=C_PLANT, label=r'True $\dot{x}$')
    axes[0].plot(t, x_hat_full[2], color=C_OBS, linestyle='--',
                 label='Full-order estimate')
    axes[0].plot(t, x_hat_red[2],  color='#9467bd', linestyle=':',
                 label='Reduced-order estimate')
    axes[0].set_ylabel(r'$\dot{x}$ (m/s)')
    axes[0].legend()

    # theta_dot is index 3
    axes[1].plot(t, x_plant[3],    color=C_PLANT, label=r'True $\dot{\theta}$')
    axes[1].plot(t, x_hat_full[3], color=C_OBS, linestyle='--',
                 label='Full-order estimate')
    axes[1].plot(t, x_hat_red[3],  color='#9467bd', linestyle=':',
                 label='Reduced-order estimate')
    axes[1].set_ylabel(r'$\dot{\theta}$ (rad/s)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()

    axes[0].set_title('Full-Order vs Reduced-Order Observer: Velocity Estimates')
    save_fig(finalize(fig), save_name)


# ============================================================================
# Optional — Feedback Linearization simulation and plot
# ============================================================================

def _rhs_fbl(t, z, A, B, C, K_fl, L, f_nl, params):
    """8-state RHS: plant + full-order observer, FL-based control."""
    from controller import fbl_control
    x     = z[:4]
    x_hat = z[4:]

    u  = fbl_control(x_hat, K_fl, params=None)
    y  = C @ x

    dx     = f_nl(t, x, u, params)
    dx_hat = A @ x_hat + B.flatten() * u + L @ (y - C @ x_hat)
    return np.concatenate([dx, dx_hat])


def simulate_fbl(x0, t_span, K_fl, L, A, B, C, f_nl,
                 params=None, n_points=2000):
    """Simulate feedback-linearization + LQR with full-order observer.

    Uses input-output FL (output y = theta, relative degree 2) to exactly
    cancel the nonlinear terms in theta_ddot, then applies a virtual LQR
    input v = -K_fl @ x_hat.

    Parameters / Returns: identical structure to simulate_stabilization.
    """
    from controller import fbl_control
    z0     = np.concatenate([x0, np.zeros(4)])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    sol = solve_ivp(
        _rhs_fbl, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-6, atol=1e-8,
        args=(A, B, C, K_fl, L, f_nl, params)
    )
    t       = sol.t
    x_plant = sol.y[:4]
    x_obs   = sol.y[4:]
    u_hist  = np.array([fbl_control(x_obs[:, i], K_fl) for i in range(len(t))])
    return t, x_plant, x_obs, u_hist


def plot_fbl_comparison(t_lqr, xp_lqr, u_lqr,
                        t_fbl, xp_fbl, u_fbl,
                        x0_label='',
                        save_name='phase2_fbl_comparison'):
    """Overlay LQR vs. FL+LQR responses for the same initial condition.

    Parameters
    ----------
    t_lqr, xp_lqr, u_lqr : LQR simulation results
    t_fbl, xp_fbl, u_fbl  : FL simulation results
    x0_label              : str — IC description for the plot title
    """
    fig, axes = new_fig(nrows=3, ncols=1, width=6.5, height=7.5, sharex=True)

    axes[0].plot(t_lqr, xp_lqr[0], color=C_PLANT, label='LQR')
    axes[0].plot(t_fbl, xp_fbl[0], color=C_REF, linestyle='--', label='FL + LQR')
    axes[0].set_ylabel(r'Position $x$ (m)')
    axes[0].legend()
    axes[0].set_title(f'LQR vs. Feedback Linearization  {x0_label}')

    axes[1].plot(t_lqr, np.degrees(xp_lqr[1]), color=C_PLANT, label='LQR')
    axes[1].plot(t_fbl, np.degrees(xp_fbl[1]), color=C_REF,
                 linestyle='--', label='FL + LQR')
    axes[1].axhline( np.degrees(0.35), color='k', linewidth=0.7,
                    linestyle=':', alpha=0.7, label=r'$\pm 20°$ limit')
    axes[1].axhline(-np.degrees(0.35), color='k', linewidth=0.7,
                    linestyle=':', alpha=0.7)
    axes[1].set_ylabel(r'Tilt $\theta$ (deg)')
    axes[1].legend()

    axes[2].plot(t_lqr, u_lqr, color=C_PLANT, label='LQR')
    axes[2].plot(t_fbl, u_fbl, color=C_REF, linestyle='--', label='FL + LQR')
    axes[2].axhline( 10, color='k', linewidth=0.7, linestyle=':', alpha=0.6,
                    label=r'$\pm u_\mathrm{max}$')
    axes[2].axhline(-10, color='k', linewidth=0.7, linestyle=':', alpha=0.6)
    axes[2].set_ylabel(r'Force $u$ (N)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()

    save_fig(finalize(fig), save_name)
