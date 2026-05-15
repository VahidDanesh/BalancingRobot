"""animate.py — Interactive side-view animation of the TWSBR.

Geometry (side view, single wheel):
  ┌──────┐  ← head circle
  │ body │  ← rectangle, pivot at wheel centre, tilts with θ
  ●       ← wheel circle, sits on ground (y = 0)
──────────── ground

Controls
--------
  ← / →  hold to apply ±5 N external force
  r       reset robot to upright with θ₀ = 0.15 rad
  q / Esc quit

The LQR + full-order observer runs in real-time.
User force is added directly to the controller output.

Usage
-----
  python animate.py
  python animate.py --theta0 0.25
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Circle, Polygon
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

from parameters import M, m, L, I, b, g
from linearize  import A, B, C
from controller import design_lqr
from observer   import design_full_observer, observer_pole_targets
from model      import f_nonlinear

# ── Visual geometry (metres) ─────────────────────────────────────────────────
R_W     = 0.055         # wheel radius
BODY_W  = 0.10          # body width
BODY_H  = 3.0 * L       # body height (CoM at L = 1/3 from bottom)
HALF_W  = BODY_W / 2

# ── Simulation ────────────────────────────────────────────────────────────────
DT      = 0.025         # seconds per animation frame
F_PUSH  = 5.0           # user-applied force magnitude (N)

# ── Colours ───────────────────────────────────────────────────────────────────
C_WHEEL  = '#2c7bb6'
C_BODY   = '#fd8d3c'
C_SPOKE  = 'white'
C_GROUND = '#444444'
C_FORCE  = '#2ca02c'
C_TRAIL  = '#c6dbef'


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _body_corners(x, theta):
    """Return (4,2) array of body rectangle corners in world coordinates.

    The rectangle has its bottom-centre pinned to (x, R_W) and is rotated
    clockwise by theta (tilt from vertical).
    """
    # Unrotated corners with bottom-centre at origin
    local = np.array([
        [-HALF_W, 0],
        [ HALF_W, 0],
        [ HALF_W, BODY_H],
        [-HALF_W, BODY_H],
    ])
    c, s = np.cos(theta), np.sin(theta)
    # Clockwise rotation by theta  →  matrix [[c, s], [-s, c]]
    R = np.array([[c, s], [-s, c]])
    rotated = local @ R.T
    return rotated + np.array([x, R_W])


def _spoke_endpoints(x, roll_angle):
    """Two spoke line endpoints for the wheel cross."""
    cx, cy = x, R_W
    pts = []
    for angle in (roll_angle, roll_angle + np.pi / 2):
        pts.append(([cx + R_W * np.cos(angle), cx - R_W * np.cos(angle)],
                    [cy + R_W * np.sin(angle), cy - R_W * np.sin(angle)]))
    return pts


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def run(x0):
    """Launch the interactive animation starting from state x0."""

    # ── Controller / observer design ─────────────────────────────────────────
    K, cl_poles      = design_lqr(A, B)
    obs_poles        = observer_pole_targets(cl_poles, factor=4.0)
    L_obs, _         = design_full_observer(A, C, obs_poles)

    # ── Mutable simulation state ──────────────────────────────────────────────
    state   = {'x': x0.copy(), 'x_hat': x0.copy(), 't': 0.0}
    u_user  = {'val': 0.0}    # force applied by user (N)

    # ── Figure layout ─────────────────────────────────────────────────────────
    VIEW_W = 0.55   # half-width of the camera window (m)
    VIEW_H = BODY_H + R_W + 0.06

    fig = plt.figure(figsize=(10, 6.5))
    ax  = fig.add_axes([0.05, 0.28, 0.90, 0.68])   # robot view
    ax_info = fig.add_axes([0.05, 0.03, 0.90, 0.20], frameon=False)
    ax_info.set_xticks([]); ax_info.set_yticks([])

    ax.set_ylim(-0.02, VIEW_H)
    ax.set_aspect('equal')
    ax.set_facecolor('#f2f2f2')
    ax.set_xlabel('Position  x  (m)', fontsize=9)
    ax.set_title('Two-Wheel Self-Balancing Robot — LQR + Observer  '
                 r'($\leftarrow\rightarrow$ keys = push,  r = reset)', fontsize=10)

    # Ground
    ground = ax.axhline(0, color=C_GROUND, lw=2.5, zorder=1)
    # Hatching below ground
    ax.fill_between([-10, 10], [-0.02, -0.02], [0, 0],
                    color='#cccccc', zorder=0)

    # Position trail (CoM path at ground level)
    trail_line, = ax.plot([], [], color=C_TRAIL, lw=1.5, zorder=2)

    # Vertical reference (dashed)
    ref_line = ax.axvline(0, color='#aaaaaa', lw=0.8, ls=':', zorder=2)

    # Wheel
    wheel_patch = Circle((0, R_W), R_W, color=C_WHEEL, zorder=4)
    ax.add_patch(wheel_patch)
    wheel_rim = Circle((0, R_W), R_W, fill=False,
                       edgecolor='white', lw=1.2, zorder=5)
    ax.add_patch(wheel_rim)

    # Spokes (two lines forming a cross)
    spoke1, = ax.plot([], [], color=C_SPOKE, lw=1.8, zorder=5)
    spoke2, = ax.plot([], [], color=C_SPOKE, lw=1.8, zorder=5)

    # Hub dot
    hub_dot, = ax.plot([0], [R_W], 'o', color='white', ms=5, zorder=6)

    # Body
    body_patch = Polygon(_body_corners(0, 0),
                         closed=True, facecolor=C_BODY,
                         edgecolor='white', lw=0.8, zorder=3)
    ax.add_patch(body_patch)

    # CoM marker
    com_dot, = ax.plot([], [], 'o', color='white', ms=4,
                       markeredgecolor='black', markeredgewidth=0.5, zorder=7)

    # Force arrow (user push)
    force_arrow = ax.annotate(
        '', xy=(0, R_W), xytext=(0, R_W),
        arrowprops=dict(arrowstyle='->', color=C_FORCE, lw=2.5),
        zorder=8
    )

    # Time / state text
    info_text = ax_info.text(
        0.01, 0.85, '', transform=ax_info.transAxes,
        fontsize=9, va='top', family='monospace'
    )
    controls_text = ax_info.text(
        0.01, 0.15,
        '← / → : push robot       r : reset       q / Esc : quit',
        transform=ax_info.transAxes, fontsize=8.5, color='#555555'
    )

    # θ bar (small vertical gauge on the right)
    ax_bar = fig.add_axes([0.92, 0.28, 0.025, 0.68])
    ax_bar.set_xlim(-1, 1); ax_bar.set_ylim(-40, 40)
    ax_bar.set_xticks([]); ax_bar.set_ylabel('θ (°)', fontsize=8)
    ax_bar.axhline(0, color='#888', lw=0.8, ls='--')
    ax_bar.axhline(20, color='#d62728', lw=0.8, ls=':')
    ax_bar.axhline(-20, color='#d62728', lw=0.8, ls=':')
    theta_bar = ax_bar.bar([0], [0], width=0.8, color='#fd8d3c',
                            bottom=0, align='center')[0]

    trail_x = []   # accumulated trail x-values

    # ── Keyboard callbacks ────────────────────────────────────────────────────
    def on_press(event):
        if event.key == 'right':
            u_user['val'] = +F_PUSH
        elif event.key == 'left':
            u_user['val'] = -F_PUSH
        elif event.key == 'r':
            state['x']     = np.array([0.0, 0.15, 0.0, 0.0])
            state['x_hat'] = np.array([0.0, 0.15, 0.0, 0.0])
            state['t']     = 0.0
            trail_x.clear()
        elif event.key in ('q', 'escape'):
            plt.close(fig)

    def on_release(event):
        if event.key in ('right', 'left'):
            u_user['val'] = 0.0

    fig.canvas.mpl_connect('key_press_event',   on_press)
    fig.canvas.mpl_connect('key_release_event', on_release)

    # ── Per-frame integration ─────────────────────────────────────────────────
    def _step():
        """Integrate plant + observer forward by DT, return (x, x_hat, u)."""
        x_now   = state['x']
        xh_now  = state['x_hat']
        t_now   = state['t']
        u_lqr   = (-K @ xh_now).item()
        u_total = np.clip(u_lqr + u_user['val'], -20.0, 20.0)

        def rhs(t, z):
            xp, xh = z[:4], z[4:]
            y      = C @ xp
            dxp    = f_nonlinear(t, xp, u_total, None)
            dxh    = A @ xh + B.flatten() * u_total + L_obs @ (y - C @ xh)
            return np.concatenate([dxp, dxh])

        sol = solve_ivp(rhs, [t_now, t_now + DT],
                        np.concatenate([x_now, xh_now]),
                        method='RK45', rtol=1e-5, atol=1e-7,
                        dense_output=False)

        z_end = sol.y[:, -1]
        state['x']     = z_end[:4]
        state['x_hat'] = z_end[4:]
        state['t']    += DT
        return state['x'], state['x_hat'], u_total

    # ── Animation update ──────────────────────────────────────────────────────
    def _update(_frame):
        x_now, _, u_total = _step()
        xi, theta_now = x_now[0], x_now[1]
        roll = xi / R_W

        # Camera follows robot
        ax.set_xlim(xi - VIEW_W, xi + VIEW_W)

        # Wheel
        wheel_patch.center = (xi, R_W)
        wheel_rim.center   = (xi, R_W)
        hub_dot.set_data([xi], [R_W])

        # Spokes
        for line, (sx, sy) in zip((spoke1, spoke2), _spoke_endpoints(xi, roll)):
            line.set_data(sx, sy)

        # Body
        body_patch.set_xy(_body_corners(xi, theta_now))

        # CoM
        com_x = xi + L * np.sin(theta_now)
        com_y = R_W + L * np.cos(theta_now)
        com_dot.set_data([com_x], [com_y])

        # Force arrow
        F_VIS = 0.12   # metres per Newton (visual scale)
        tip_x = xi + u_total * F_VIS
        force_arrow.set_position((xi, R_W))
        force_arrow.xy = (tip_x, R_W)

        # Trail
        trail_x.append(xi)
        trail_line.set_data(trail_x, [R_W] * len(trail_x))

        # θ bar
        deg = np.degrees(theta_now)
        theta_bar.set_height(deg)
        theta_bar.set_y(min(deg, 0))
        theta_bar.set_facecolor('#d62728' if abs(deg) > 20 else '#fd8d3c')

        # Info text
        info_text.set_text(
            f"t = {state['t']:6.2f} s    "
            f"x = {xi:+.3f} m    "
            f"θ = {deg:+6.2f}°    "
            f"u_LQR = {(-K @ state['x_hat']).item():+6.2f} N    "
            f"u_push = {u_user['val']:+.1f} N"
        )

        return (wheel_patch, wheel_rim, spoke1, spoke2, hub_dot,
                body_patch, com_dot, trail_line, info_text, theta_bar)

    anim = FuncAnimation(fig, _update, interval=int(DT * 1000),
                         blit=False, cache_frame_data=False)
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Interactive TWSBR animation')
    parser.add_argument('--theta0', type=float, default=0.15,
                        help='Initial tilt angle (rad), default 0.15')
    args = parser.parse_args()

    x0 = np.array([0.0, args.theta0, 0.0, 0.0])
    print(f"Starting: θ₀ = {args.theta0} rad ({np.degrees(args.theta0):.1f}°)")
    print("Controls: ← → push | r reset | q quit")
    run(x0)
