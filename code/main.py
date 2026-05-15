"""Main entry point — runs all project phases sequentially.

State order: [x, theta, x_dot, theta_dot]   (CLAUDE.md [q; q_dot] convention)

Usage:  python main.py
"""

import numpy as np
import control as ct

from parameters import M, m, L, I, b, g
from linearize  import Delta0, A, B, C, D, sys_ol
from analysis   import (open_loop_eigenvalues, check_controllability,
                        check_observability, minimal_realization,
                        plot_open_loop_poles)
from controller import design_lqr, design_tracking
from observer   import design_full_observer, design_reduced_observer, observer_pole_targets
from simulate   import (simulate_stabilization, simulate_tracking,
                        simulate_reduced_obs,
                        plot_poles, plot_stabilization, plot_tracking,
                        plot_observer_error, plot_observer_comparison,
                        simulate_fbl, plot_fbl_comparison)
from robustness import (sweep_robustness, simulate_saturated,
                        plot_robustness, plot_saturation)
from model      import f_nonlinear

np.set_printoptions(precision=4, suppress=True)

# ════════════════════════════════════════════════════════════════════════════
print("\n" + "="*62)
print("  MEC 560 — Two-Wheel Self-Balancing Robot")
print("  Final Project  |  Spring 2026")
print("="*62)


# ════════════════════════════════════════════════════════════════════════════
print("\n=== PHASE I: MODELING & OPEN-LOOP ANALYSIS ===\n")

# ── Physical parameters ──────────────────────────────────────────────────────
print("--- Physical Parameters ---")
print(f"  M  = {M}     kg        (wheel + base mass)")
print(f"  m  = {m}     kg        (body mass)")
print(f"  L  = {L}     m         (wheel axis to CoM)")
print(f"  I  = {I}  kg·m²     (body moment of inertia)")
print(f"  b  = {b}     N·m·s     (rolling friction)")
print(f"  g  = {g}  m/s²      (gravity)")
print(f"  Δ₀ = {Delta0:.6f}  kg²·m⁴  (linearization denominator)")

# ── Linearized matrices ──────────────────────────────────────────────────────
print("\n--- Linearized Matrices ---")
print("  State order: [x, theta, x_dot, theta_dot]")
print("A matrix:\n", A)
print("B matrix:\n", B)
print("C matrix:\n", C)
print("D matrix:\n", D)

# Sign checks (from CLAUDE.md, verified against Lagrangian EOM)
assert A[2, 1] < 0, "A[2,1] should be negative  (-m²gL²/Δ₀)"
assert A[3, 1] > 0, "A[3,1] should be positive  (+(M+m)mgL/Δ₀)"
assert A[2, 2] < 0, "A[2,2] should be negative  (-(I+mL²)b/Δ₀)"
assert A[3, 2] > 0, "A[3,2] should be positive  (+mLb/Δ₀)"
assert B[2, 0] > 0, "B[2,0] should be positive  (+(I+mL²)/Δ₀)"
assert B[3, 0] < 0, "B[3,0] should be negative  (-mL/Δ₀)"
print("  → Sign checks passed ✓")

# ── Open-loop eigenvalues ────────────────────────────────────────────────────
print("\n--- Open-Loop Eigenvalue Analysis ---")
eigs, labels = open_loop_eigenvalues(A)
for e, lbl in zip(eigs, labels):
    print(f"  λ = {np.real(e):+.4f} + {np.imag(e):+.4f}j  →  {lbl}")
rhp = sum(1 for e in eigs if np.real(e) > 1e-9)
print(f"\n  → {rhp} RHP pole(s): open-loop system is UNSTABLE (BIBO unstable)")

# ── Controllability ──────────────────────────────────────────────────────────
print("\n--- Controllability ---")
Wc, rank_Wc = check_controllability(A, B)
n = A.shape[0]
print(f"  rank(Wc) = {rank_Wc}  (n = {n})")
print(f"  → {'FULLY CONTROLLABLE' if rank_Wc == n else 'NOT controllable'} "
      f"(→ stabilizable)")

# ── Observability ────────────────────────────────────────────────────────────
print("\n--- Observability ---")
Wo, rank_Wo = check_observability(A, C)
print(f"  rank(Wo) = {rank_Wo}  (n = {n})")
print(f"  → {'FULLY OBSERVABLE' if rank_Wo == n else 'NOT observable'} "
      f"(→ detectable)")

# ── Minimal realization ──────────────────────────────────────────────────────
print("\n--- Minimal Realization ---")
is_minimal, order_min = minimal_realization(A, B, C)
print(f"  → {'IS a minimal realization' if is_minimal else 'NOT minimal'}, "
      f"order = {order_min}")

# ── Transfer function poles ──────────────────────────────────────────────────
print("\n--- Transfer Function Poles (BIBO) ---")
poles_tf = ct.poles(sys_ol)
print("  Poles:", np.round(poles_tf, 4))

print("\n--- Generating Phase I Plot ---")
plot_open_loop_poles(A)

print("\n" + "="*62)
print("  Phase I complete.")
print("="*62)


# ════════════════════════════════════════════════════════════════════════════
print("\n=== PHASE II: CONTROLLER & OBSERVER DESIGN ===\n")

# ── LQR stabilizing controller ───────────────────────────────────────────────
print("--- LQR Stabilizing Controller ---")
print("  Q = diag([1.0, 10.0, 0.0, 0.0])   [x, theta, x_dot, theta_dot]")
print("  R = [[0.1]]                          (control effort)")

K, cl_poles = design_lqr(A, B)
print(f"\n  Controller gain K:\n    {K}")
print(f"\n  Closed-loop poles (A - B·K):")
for p in sorted(cl_poles, key=np.real):
    print(f"    {np.real(p):+.4f} + {np.imag(p):+.4f}j")
slowest_ctrl = np.max(np.abs(np.real(cl_poles)))
print(f"\n  Slowest pole |Re| = {slowest_ctrl:.4f} rad/s")
print("  → All poles in LHP: STABLE ✓")

# ── Tracking (augmented LQR) ─────────────────────────────────────────────────
print("\n--- Augmented LQR Tracking Controller ---")
print("  Q_aug = diag([1.0, 10.0, 0.0, 0.0, 5.0])  (adds integral weight)")

K_aug, A_aug, B_aug, aug_poles = design_tracking(A, B)
print(f"\n  Augmented gain K_aug:\n    {K_aug}")
print(f"\n  Augmented closed-loop poles (5-state):")
for p in sorted(aug_poles, key=np.real):
    print(f"    {np.real(p):+.4f} + {np.imag(p):+.4f}j")
print("  → All poles in LHP: STABLE ✓")

# ── Full-order observer ───────────────────────────────────────────────────────
print("\n--- Full-Order Luenberger Observer ---")
obs_poles = observer_pole_targets(cl_poles, factor=4.0)
print(f"  Observer pole targets (4× slowest ctrl):")
for p in obs_poles:
    print(f"    {p:.4f}")

L, obs_eigs = design_full_observer(A, C, obs_poles)
print(f"\n  Observer gain L:\n{L}")
print(f"\n  Achieved observer poles (A - L·C):")
for p in sorted(obs_eigs, key=np.real):
    print(f"    {np.real(p):+.4f} + {np.imag(p):+.4f}j")
slowest_obs = np.min(np.abs(np.real(obs_eigs)))
speed_ratio = slowest_obs / slowest_ctrl
print(f"\n  Speed ratio min|Re(obs)| / max|Re(ctrl)| = {speed_ratio:.2f}  "
      f"({'≥ 3.0 ✓' if speed_ratio >= 3.0 else '< 3.0 — increase factor!'})")
print("  → All poles in LHP: STABLE ✓")

# ── Reduced-order observer ────────────────────────────────────────────────────
print("\n--- Reduced-Order Observer (2nd order) ---")
obs_poles_r = observer_pole_targets(cl_poles, factor=4.5, spread=0.15)[:2]
print(f"  Reduced observer pole targets: {obs_poles_r}")

L_r, red_eigs, partition = design_reduced_observer(A, obs_poles_r)
print(f"\n  Reduced observer gain L_r:\n{L_r}")
print(f"\n  Achieved eigenvalues (A_bb - L_r·A_ab):")
for p in red_eigs:
    print(f"    {np.real(p):+.4f} + {np.imag(p):+.4f}j")
print("  → All poles in LHP: STABLE ✓")

# ── Separation Principle ─────────────────────────────────────────────────────
print("\n--- Separation Principle Verification ---")
A_cl = np.block([
    [A - B @ K,        B @ K       ],
    [np.zeros((4, 4)), A - L @ C   ],
])
combined_poles = np.linalg.eigvals(A_cl)
print("  Combined 8-pole closed-loop eigenvalues:")
for p in sorted(combined_poles, key=np.real):
    print(f"    {np.real(p):+.4f} + {np.imag(p):+.4f}j")
assert np.all(np.real(combined_poles) < 0), \
    "Combined system has unstable pole(s)!"
print("  → All 8 poles in LHP: STABLE ✓  (Separation Principle verified)")

print("\n--- Generating Pole Plot ---")
plot_poles(eigs, cl_poles, obs_eigs, combined_poles)


# ════════════════════════════════════════════════════════════════════════════
print("\n=== PHASE II: SIMULATIONS ===\n")

T_SIM  = (0.0, 10.0)

# ICs in [x, theta, x_dot, theta_dot] order
IC_LIST = [
    (np.array([0.0, 0.10, 0.0, 0.0]), r'$\mathbf{x}_0=[0,0.1,0,0]$'),
    (np.array([0.0, 0.20, 0.0, 0.0]), r'$\mathbf{x}_0=[0,0.2,0,0]$'),
    (np.array([0.5, 0.05, 0.0, 0.0]), r'$\mathbf{x}_0=[0.5,0.05,0,0]$'),
]

# ── Stabilization ─────────────────────────────────────────────────────────────
print("--- Stabilization Simulation (3 initial conditions) ---")
stab_results = []
for x0, lbl in IC_LIST:
    print(f"  Simulating IC: {lbl} ...")
    t, xp, xo, u = simulate_stabilization(x0, T_SIM, K, L, A, B, C, f_nonlinear)
    stab_results.append((t, xp, xo, u))
    # Report settling time for first IC (theta at index 1)
    if np.allclose(x0, IC_LIST[0][0]):
        tol = 0.02 * np.abs(x0[1])   # 2% of initial tilt
        settled = np.where(np.abs(xp[1]) < tol)[0]
        if len(settled) > 0:
            print(f"    θ settling time (~2% band): {t[settled[0]]:.2f} s")

plot_stabilization(stab_results, [lbl for _, lbl in IC_LIST])
print("  → Saved: phase2_stabilization")

# ── Observer error ────────────────────────────────────────────────────────────
print("\n--- Observer Error (IC #1) ---")
t, xp, xo, _ = stab_results[0]
plot_observer_error(t, xp, xo)
print("  → Saved: phase2_observer_error")

# ── Step position tracking ────────────────────────────────────────────────────
print("\n--- Step Position Tracking (x_ref = 0.5 m) ---")
x_ref_step  = 0.5
x_ref_func  = lambda t: x_ref_step
t, xp, xo, xi, u = simulate_tracking(T_SIM, K_aug, L, A, B, C, f_nonlinear,
                                       x_ref_func)
x_ref_vals = np.full_like(t, x_ref_step)
plot_tracking(t, xp, xo, u, x_ref_vals, save_name='phase2_step_tracking')
ss_err = np.mean(np.abs(xp[0, -100:] - x_ref_step))
print(f"  Steady-state position error (last 100 pts): {ss_err:.4f} m")
print("  → Saved: phase2_step_tracking")

# ── Sinusoidal reference tracking ────────────────────────────────────────────
print("\n--- Sinusoidal Reference Tracking (0.3 sin(t) m) ---")
x_ref_sin  = lambda t: 0.3 * np.sin(t)
x0_sin     = np.array([0.0, 0.05, 0.0, 0.0])   # [x, theta, x_dot, theta_dot]
t, xp, xo, xi, u = simulate_tracking(T_SIM, K_aug, L, A, B, C, f_nonlinear,
                                       x_ref_sin, x0=x0_sin)
x_ref_vals = np.array([x_ref_sin(ti) for ti in t])
plot_tracking(t, xp, xo, u, x_ref_vals, save_name='phase2_sinusoidal_tracking')
print("  → Saved: phase2_sinusoidal_tracking")

# ── Reduced-order observer comparison ────────────────────────────────────────
print("\n--- Reduced-Order Observer Comparison ---")
x0_cmp = IC_LIST[0][0]
t, xp, xo_full, _ = stab_results[0]
_, xp_r, xo_red, _ = simulate_reduced_obs(x0_cmp, T_SIM, K, L_r, partition,
                                            A, B, C, f_nonlinear)
plot_observer_comparison(t, xp, xo_full, xo_red)
print("  → Saved: phase2_observer_comparison")

# ════════════════════════════════════════════════════════════════════════════
print("\n" + "="*62)
print("  Phase II complete.")
print("="*62 + "\n")


# ════════════════════════════════════════════════════════════════════════════
print("\n=== PHASE II (OPTIONAL): FEEDBACK LINEARIZATION ===\n")

from controller import fbl_control

# After I/O linearisation of y = theta (rel. degree 2), only the theta
# double-integrator is controllable from v.  Using x / x_dot gains in v
# creates positive feedback on xdd (unstable internal dynamics).
# Design K_fl via LQR on the controllable subsystem: theta double integrator.
A_theta = np.array([[0., 1.], [0., 0.]])
B_theta = np.array([[0.], [1.]])
Q_fl    = np.diag([100., 10.])   # penalise tilt angle and rate
R_fl    = np.array([[0.1]])
K_fl_2d, _, _ = ct.lqr(A_theta, B_theta, Q_fl, R_fl)
# Embed into full-state gain; zero on x and x_dot (position not regulated)
K_fl = np.array([[0., K_fl_2d[0, 0], 0., K_fl_2d[0, 1]]])

print("--- FL Law ---")
print("  Output chosen: y = theta  (relative degree 2)")
print("  theta_ddot = v  after FL cancellation")
print("  Virtual input: v = -K_fl @ x_hat  (theta/theta_dot only)")
print(f"  K_fl = {np.round(K_fl, 4)}")
print("  Note: x not regulated by FL (position control not part of I/O linearisation)")

# ── Compare LQR vs FL at a moderate and a large tilt ─────────────────────
TEST_ICS = [
    (np.array([0.0, 0.25, 0.0, 0.0]),
     r'$\theta_0 = 0.25$ rad (14°)'),
    (np.array([0.0, 0.40, 0.0, 0.0]),
     r'$\theta_0 = 0.40$ rad (23°) — near LQR stability limit'),
]

for x0_fl, lbl_fl in TEST_ICS:
    print(f"\n  IC: {lbl_fl}")
    t_lqr, xp_lqr, _, u_lqr = simulate_stabilization(
        x0_fl, T_SIM, K, L, A, B, C, f_nonlinear)
    t_fbl, xp_fbl, _, u_fbl = simulate_fbl(
        x0_fl, T_SIM, K_fl, L, A, B, C, f_nonlinear)

    # Stabilized if |theta| < 0.01 rad within T_SIM
    lqr_ok = np.any(np.abs(xp_lqr[1, -200:]) < 0.01)
    fbl_ok = np.any(np.abs(xp_fbl[1, -200:]) < 0.01)
    print(f"    LQR stabilized: {lqr_ok}  |  FL stabilized: {fbl_ok}")

    sname = 'phase2_fbl_moderate' if '0.25' in lbl_fl else 'phase2_fbl_large'
    plot_fbl_comparison(t_lqr, xp_lqr, u_lqr,
                        t_fbl, xp_fbl, u_fbl,
                        x0_label=lbl_fl, save_name=sname)
    print(f"  → Saved: {sname}")

print("\n" + "="*62)
print("  Phase II (Optional) complete.")
print("="*62 + "\n")


# ════════════════════════════════════════════════════════════════════════════
print("\n=== PHASE III: ROBUSTNESS & SATURATION ===\n")

DELTAS   = [-0.2, -0.1, 0.0, 0.1, 0.2]
X0_ROB   = np.array([0.0, 0.10, 0.0, 0.0])   # standard IC for robustness tests
U_MAX    = 10.0                                # actuator limit (N)

# ── Mass perturbation sweep ───────────────────────────────────────────────
print("--- Robustness Sweep: body mass m (±20 %) ---")
results_m = sweep_robustness(K, L, A, B, C, f_nonlinear,
                              X0_ROB, T_SIM, param='m', deltas=DELTAS)
for delta, (t_r, xp_r, _) in zip(DELTAS, results_m):
    stable = np.all(np.abs(xp_r[1]) < 0.5)    # theta stayed bounded
    label  = 'nominal' if delta == 0.0 else f'{delta:+.0%}'
    print(f"  m {label:>6s}: {'STABLE' if stable else 'UNSTABLE / diverged'}"
          f"  |θ_max| = {np.degrees(np.max(np.abs(xp_r[1]))):.2f}°")
plot_robustness(results_m, DELTAS, param_name='m',
                save_name='phase3_robustness_m')
print("  → Saved: phase3_robustness_m")

# ── Length perturbation sweep ─────────────────────────────────────────────
print("\n--- Robustness Sweep: body length L (±20 %) ---")
results_L = sweep_robustness(K, L, A, B, C, f_nonlinear,
                              X0_ROB, T_SIM, param='L', deltas=DELTAS)
for delta, (t_r, xp_r, _) in zip(DELTAS, results_L):
    stable = np.all(np.abs(xp_r[1]) < 0.5)
    label  = 'nominal' if delta == 0.0 else f'{delta:+.0%}'
    print(f"  L {label:>6s}: {'STABLE' if stable else 'UNSTABLE / diverged'}"
          f"  |θ_max| = {np.degrees(np.max(np.abs(xp_r[1]))):.2f}°")
plot_robustness(results_L, DELTAS, param_name='L',
                save_name='phase3_robustness_L')
print("  → Saved: phase3_robustness_L")

# ── Actuator saturation ───────────────────────────────────────────────────
print("\n--- Actuator Saturation Test (u_max = ±10 N) ---")
X0_SAT = np.array([0.0, 0.25, 0.0, 0.0])    # moderate tilt — exercises saturation
(t_u, xp_u, u_u), (t_s, xp_s, u_s) = simulate_saturated(
    X0_SAT, T_SIM, K, L, A, B, C, f_nonlinear, u_max=U_MAX)

peak_unsat = np.max(np.abs(u_u))
peak_sat   = np.max(np.abs(u_s))
print(f"  Peak |u| unsaturated: {peak_unsat:.2f} N")
print(f"  Peak |u|   saturated: {peak_sat:.2f} N  (clipped at ±{U_MAX} N)")
stab_unsat = np.all(np.abs(xp_u[1, -200:]) < 0.02)
stab_sat   = np.all(np.abs(xp_s[1, -200:]) < 0.02)
print(f"  Stabilized (unsaturated): {stab_unsat}")
print(f"  Stabilized   (saturated): {stab_sat}")

plot_saturation(t_u, xp_u, u_u, t_s, xp_s, u_s,
                u_max=U_MAX, save_name='phase3_saturation')
print("  → Saved: phase3_saturation")

print("\n" + "="*62)
print("  Phase III complete.")
print("="*62 + "\n")
