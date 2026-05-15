"""Phase III — Robustness Analysis and Actuator Saturation.

Two analyses:
    1. Parameter uncertainty sweep: perturb m or L by delta ∈ {-0.2,-0.1,0,0.1,0.2}.
       Nominal K and L are kept fixed; only the PLANT sees the perturbed dynamics.
    2. Actuator saturation: clip control input to ±u_max and compare with
       the unsaturated case.

Exports
-------
apply_saturation(u, u_max)
build_perturbed_AB(delta_m, delta_L)
sweep_robustness(K, L_obs, A, B, C, f_nl, x0, t_span, param, deltas)
simulate_saturated(x0, t_span, K, L_obs, A, B, C, f_nl, u_max)
plot_robustness(sweep_results, deltas, param_name, save_name)
plot_saturation(t_unsat, xp_unsat, u_unsat, t_sat, xp_sat, u_sat, u_max, save_name)
"""

import numpy as np
from scipy.integrate import solve_ivp

from parameters import M, m, L, I, b, g
from plot_utils import C_REF, new_fig, save_fig, finalize, C_PLANT, C_CTRL, PALETTE


# ============================================================================
# Helpers
# ============================================================================

def apply_saturation(u, u_max=10.0):
    """Clip control input to ±u_max (actuator saturation model)."""
    return float(np.clip(u, -u_max, u_max))


def build_perturbed_AB(delta_m=0.0, delta_L=0.0):
    """Return A, B matrices for perturbed physical parameters.

    The perturbation is multiplicative:
        m_pert = m * (1 + delta_m)
        L_pert = L * (1 + delta_L)
    All other parameters (M, I, b, g) remain at their nominal values.

    Parameters
    ----------
    delta_m : float — fractional perturbation of body mass m  (e.g. 0.2 = +20 %)
    delta_L : float — fractional perturbation of body length L (e.g. -0.1 = -10 %)

    Returns
    -------
    A_pert : ndarray (4,4)
    B_pert : ndarray (4,1)
    params  : dict — perturbed parameter dict for f_nonlinear
    """
    m_p = m * (1.0 + delta_m)
    L_p = L * (1.0 + delta_L)

    D0 = (M + m_p) * (I + m_p * L_p**2) - (m_p * L_p)**2

    A_pert = np.array([
        [0,  0,                                   1,                            0],
        [0,  0,                                   0,                            1],
        [0, -m_p**2 * L_p**2 * g / D0,           -(I + m_p*L_p**2)*b / D0,    0],
        [0,  (M + m_p) * m_p * g * L_p / D0,      m_p * L_p * b / D0,         0],
    ])

    B_pert = np.array([
        [0],
        [0],
        [ (I + m_p * L_p**2) / D0],
        [-(m_p * L_p) / D0],
    ])

    params = {'M': M, 'm': m_p, 'L': L_p, 'I': I, 'b': b, 'g': g}
    return A_pert, B_pert, params


# ============================================================================
# Internal RHS for robustness simulations
# ============================================================================

def _rhs_robust(t, z, K, L_obs, A_nom, B_nom, C, f_nl, params, u_max=None):
    """8-state closed-loop RHS with optional saturation.

    Plant uses perturbed params; observer uses nominal A, B (no model update).
    """
    x_plant = z[:4]
    x_obs   = z[4:]

    u_raw = -(K @ x_obs).item()
    u = float(np.clip(u_raw, -u_max, u_max)) if u_max is not None else u_raw

    y = C @ x_plant
    dx      = f_nl(t, x_plant, u, params)
    dx_obs  = A_nom @ x_obs + B_nom.flatten() * u + L_obs @ (y - C @ x_obs)

    return np.concatenate([dx, dx_obs])


# ============================================================================
# Public functions
# ============================================================================

def sweep_robustness(K, L_obs, A_nom, B_nom, C, f_nl, x0,
                     t_span, param='m', deltas=None, n_points=2000):
    """Simulate closed-loop response for a range of parameter perturbations.

    Parameters
    ----------
    K, L_obs : nominal controller and observer gains
    A_nom, B_nom : nominal linearized matrices (used in observer)
    C        : output matrix
    f_nl     : nonlinear plant dynamics f(t, x, u, params)
    x0       : ndarray (4,) — initial condition
    t_span   : (t0, tf)
    param    : 'm' or 'L' — which parameter to perturb
    deltas   : list of fractional perturbations; defaults to [-0.2,-0.1,0,0.1,0.2]

    Returns
    -------
    list of (t, x_plant, u_hist) for each delta value
    """
    if deltas is None:
        deltas = [-0.2, -0.1, 0.0, 0.1, 0.2]

    results = []
    z0 = np.concatenate([x0, np.zeros(4)])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    for delta in deltas:
        dm = delta if param == 'm' else 0.0
        dL = delta if param == 'L' else 0.0
        _, _, params_pert = build_perturbed_AB(dm, dL)

        sol = solve_ivp(
            _rhs_robust, t_span, z0, method='RK45',
            t_eval=t_eval, rtol=1e-6, atol=1e-8,
            args=(K, L_obs, A_nom, B_nom, C, f_nl, params_pert, None)
        )
        t       = sol.t
        x_plant = sol.y[:4]
        u_hist  = -(K @ sol.y[4:]).flatten()
        results.append((t, x_plant, u_hist))

    return results


def simulate_saturated(x0, t_span, K, L_obs, A, B, C, f_nl,
                       u_max=10.0, n_points=2000):
    """Simulate closed-loop with and without actuator saturation.

    Returns two result tuples (unsaturated, saturated).
    Each tuple: (t, x_plant, u_hist).
    """
    z0     = np.concatenate([x0, np.zeros(4)])
    t_eval = np.linspace(t_span[0], t_span[1], n_points)

    params_nom = {'M': M, 'm': m, 'L': L, 'I': I, 'b': b, 'g': g}

    # Unsaturated
    sol_u = solve_ivp(
        _rhs_robust, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-8, atol=1e-10,
        args=(K, L_obs, A, B, C, f_nl, params_nom, None)
    )
    t_u  = sol_u.t
    xp_u = sol_u.y[:4]
    u_u  = -(K @ sol_u.y[4:]).flatten()

    # Saturated
    sol_s = solve_ivp(
        _rhs_robust, t_span, z0, method='RK45',
        t_eval=t_eval, rtol=1e-8, atol=1e-10,
        args=(K, L_obs, A, B, C, f_nl, params_nom, u_max)
    )
    t_s  = sol_s.t
    xp_s = sol_s.y[:4]
    # Recompute saturated u from observer states
    u_s  = np.clip(-(K @ sol_s.y[4:]).flatten(), -u_max, u_max)

    return (t_u, xp_u, u_u), (t_s, xp_s, u_s)


# ============================================================================
# Plotting
# ============================================================================

def plot_robustness(sweep_results, deltas, param_name='m',
                    save_name='phase3_robustness_m'):
    """Overlay theta(t) for all perturbation levels.

    Parameters
    ----------
    sweep_results : list of (t, x_plant, u_hist) from sweep_robustness
    deltas        : list of delta values matching sweep_results
    param_name    : 'm' or 'L' — for axis label
    save_name     : output filename (no extension)
    """
    sym = r'$m$' if param_name == 'm' else r'$L$'
    fig, axes = new_fig(2, 1, height=5.5, sharex=True)
    ax_th, ax_u = axes

    nom_idx = deltas.index(0.0) if 0.0 in deltas else None

    for i, (delta, (t, xp, u)) in enumerate(zip(deltas, sweep_results)):
        color = PALETTE[i % len(PALETTE)]
        lw    = 2.0 if delta == 0.0 else 1.2
        ls    = '-' if delta == 0.0 else '--'
        lbl   = (f'{sym} nominal' if delta == 0.0
                 else f'{sym} {delta:+.0%}')
        ax_th.plot(t, np.degrees(xp[1]), color=color, lw=lw, ls=ls, label=lbl)
        ax_u.plot(t, u, color=color, lw=lw, ls=ls, label=lbl)

    ax_th.axhline(0, color='k', lw=0.6, ls=':')
    ax_th.set_ylabel(r'Tilt angle $\theta$ (°)')
    ax_th.legend(fontsize=7.5, ncol=2)
    ax_th.set_title(f'Robustness Sweep — {sym} Perturbation')

    ax_u.axhline( 10, color='gray', lw=0.8, ls=':', label='±10 N limit')
    ax_u.axhline(-10, color='gray', lw=0.8, ls=':')
    ax_u.set_ylabel(r'Control input $u$ (N)')
    ax_u.set_xlabel('Time (s)')
    ax_u.legend(fontsize=7.5, ncol=2)

    save_fig(finalize(fig), save_name)


def plot_saturation(t_unsat, xp_unsat, u_unsat,
                    t_sat,   xp_sat,   u_sat,
                    u_max=10.0,
                    save_name='phase3_saturation'):
    """Compare saturated vs unsaturated closed-loop response.

    Two rows: tilt angle theta(t) and control input u(t).
    """
    fig, axes = new_fig(3, 1, height=7.0, sharex=True)
    ax_x, ax_th, ax_u = axes

    # Position
    ax_x.plot(t_unsat, xp_unsat[0], color=C_PLANT,  lw=1.8, label='Unsaturated')
    ax_x.plot(t_sat,   xp_sat[0],   color=C_REF, lw=1.8, ls='--', label='Saturated')
    ax_x.axhline(0, color='k', lw=0.6, ls=':')
    ax_x.set_ylabel(r'Position $x$ (m)')
    ax_x.legend()
    ax_x.set_title('Actuator Saturation: Saturated vs. Unsaturated Response')

    # Tilt
    ax_th.plot(t_unsat, np.degrees(xp_unsat[1]), color=C_PLANT,  lw=1.8, label='Unsaturated')
    ax_th.plot(t_sat,   np.degrees(xp_sat[1]),   color=C_REF, lw=1.8, ls='--', label='Saturated')
    ax_th.axhline(0, color='k', lw=0.6, ls=':')
    ax_th.set_ylabel(r'Tilt angle $\theta$ (°)')
    ax_th.legend()

    # Control
    ax_u.plot(t_unsat, u_unsat, color=C_PLANT,  lw=1.8, label='Unsaturated $u$')
    ax_u.plot(t_sat,   u_sat,   color=C_REF, lw=1.8, ls='--', label='Saturated $u$')
    ax_u.axhline( u_max, color='gray', lw=1.0, ls=':', label=f'±{u_max:.0f} N limit')
    ax_u.axhline(-u_max, color='gray', lw=1.0, ls=':')
    ax_u.set_ylabel(r'Control input $u$ (N)')
    ax_u.set_xlabel('Time (s)')
    ax_u.legend()

    save_fig(finalize(fig), save_name)
