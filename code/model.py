"""Nonlinear equations of motion for the two-wheel self-balancing robot.

State: x = [x, theta, x_dot, theta_dot]  shape (4,)   — [q; q_dot] order
Input: u  — horizontal force at wheel contact  (N)

EOM (Lagrangian, solved via mass-matrix inverse):
    M(theta) = [[M+m, +mL*cos(theta)],
                [+mL*cos(theta), I+mL²]]

    M(theta)^{-1} = (1/Delta) * [[I+mL², -mL*cos(theta)],
                                   [-mL*cos(theta), M+m]]

    phi = u - b*x_dot + m*L*theta_dot²*sin(theta)

    xdd = [(I+mL²)*phi - m²gL²*sin(theta)*cos(theta)] / Delta
    tdd = [(M+m)*m*g*L*sin(theta) - m*L*cos(theta)*phi]  / Delta

    Delta(theta) = (M+m)*(I+mL²) - (mL*cos(theta))²
"""

import numpy as np
from parameters import M, m, L, I, b, g


def f_nonlinear(t, x, u_func, params=None):
    """Compute dx/dt for the nonlinear plant.

    Parameters
    ----------
    t       : float      — current time (s)
    x       : array (4,) — state [x, theta, x_dot, theta_dot]
    u_func  : callable or scalar — control input; if callable, called as u_func(t, x)
    params  : dict or None — override physical parameters (for robustness sweeps)

    Returns
    -------
    dxdt : array (4,) — [x_dot, theta_dot, x_ddot, theta_ddot]
    """
    _M = params['M'] if params and 'M' in params else M
    _m = params['m'] if params and 'm' in params else m
    _L = params['L'] if params and 'L' in params else L
    _I = params['I'] if params and 'I' in params else I
    _b = params['b'] if params and 'b' in params else b
    _g = params['g'] if params and 'g' in params else g

    x_pos, theta, x_vel, theta_dot = x   # [q; q_dot] ordering
    u = float(u_func(t, x)) if callable(u_func) else float(u_func)

    # Angle-dependent denominator
    delta_0 = (_M + _m) * (_I + _m*_L**2) - (_m*_L*np.cos(theta))**2

    # Generalized force for the cart coordinate (M⁻¹ row-1 numerator)
    phi = u - _b*x_vel + _m*_L*theta_dot**2 * np.sin(theta)

    xdd = ((_I + _m*_L**2) * phi
           - _m**2 * _L**2 * _g * np.sin(theta) * np.cos(theta)) / delta_0

    tdd = ((_M + _m) * _m*_g*_L * np.sin(theta)
           - _m*_L * np.cos(theta) * phi) / delta_0

    # Return derivative of state [x, theta, x_dot, theta_dot]
    return np.array([x_vel, theta_dot, xdd, tdd])
