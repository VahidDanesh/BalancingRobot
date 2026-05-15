"""Jacobian linearization of the TWSBR about the upright equilibrium.

State order (CLAUDE.md [q; q_dot] convention):
    x_state = [x, theta, x_dot, theta_dot]   indices [0, 1, 2, 3]

This ordering yields the clean block structure:
    A = [[0_2,          I_2        ],
         [-M0_inv @ K,  -M0_inv @ C0]]
    B = [[0_2            ],
         [M0_inv @ B_u   ]]

where:
    M0    = [[M+m, mL], [mL, I+mL²]]          (linearized mass matrix)
    C0    = [[b, 0], [0, 0]]                   (linearized damping)
    K_mat = [[0, 0], [0, -mgL]]               (linearized stiffness, K[1,1]<0 → unstable)
    B_u   = [1, 0]^T

    M0_inv = (1/Delta0) * [[I+mL², -mL], [-mL, M+m]]

Equilibrium: x* = [0, 0, 0, 0],  u* = 0
Small-angle: sin(theta)≈theta, cos(theta)≈1, theta_dot²≈0

Exports: Delta0, A, B, C, D, sys_ol
"""

import numpy as np
import control as ct
from parameters import M, m, L, I, b, g

# Linearization denominator  Δ₀ = det(M0)
Delta0 = (M + m) * (I + m*L**2) - (m*L)**2

# ── State matrix A  (4×4)
# Block form: top = [0 | I],  bottom = [-M0_inv@K | -M0_inv@C0]
#
# -M0_inv @ K_mat  (bottom-left 2×2, K_mat[1,1] = -mgL):
#   row 0: [0,  -m²gL²/Δ₀]
#   row 1: [0, +(M+m)mgL/Δ₀]
#
# -M0_inv @ C0  (bottom-right 2×2):
#   row 0: [-(I+mL²)b/Δ₀,  0]
#   row 1: [+mLb/Δ₀,         0]
A = np.array([
    [0,  0,                           1,                          0],
    [0,  0,                           0,                          1],
    [0, -m**2 * L**2 * g / Delta0,   -(I + m*L**2)*b / Delta0,  0],
    [0,  (M + m)*m*g*L / Delta0,      m*L*b / Delta0,            0],
])

# ── Input matrix B  (4×1)
# Top block: 0_2.  Bottom block: M0_inv @ B_u = [+(I+mL²)/Δ₀, -mL/Δ₀]^T
B = np.array([
    [0],
    [0],
    [ (I + m*L**2) / Delta0],
    [-(m*L) / Delta0],
])

# ── Output matrix C  (2×4)
# y = [x, theta] — first two states directly → C = [I_2 | 0_2]
C = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
])

# ── Feedthrough matrix D  (2×1)
D = np.zeros((2, 1))

# ── Open-loop state-space object
sys_ol = ct.ss(A, B, C, D)
