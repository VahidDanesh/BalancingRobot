"""Phase I analysis: eigenvalues, controllability, observability, minimal realization."""

import numpy as np
import control as ct


def open_loop_eigenvalues(A):
    """Return eigenvalues of A and a stability classification string per mode."""
    eigs = np.linalg.eigvals(A)
    labels = []
    for e in eigs:
        if np.real(e) > 1e-9:
            labels.append("UNSTABLE (RHP)")
        elif np.real(e) < -1e-9:
            labels.append("stable  (LHP)")
        else:
            labels.append("marginal (imag axis)")
    return eigs, labels


def check_controllability(A, B):
    """Return controllability matrix and its rank."""
    Wc = ct.ctrb(A, B)
    rank = np.linalg.matrix_rank(Wc)
    return Wc, rank


def check_observability(A, C):
    """Return observability matrix and its rank."""
    Wo = ct.obsv(A, C)
    rank = np.linalg.matrix_rank(Wo)
    return Wo, rank


def minimal_realization(A, B, C):
    """Determine if the system is a minimal realization from rank tests.

    A system is minimal iff it is both controllable and observable.
    Returns (is_minimal, order) without requiring slycot.
    """
    n = A.shape[0]
    _, rank_c = check_controllability(A, B)
    _, rank_o = check_observability(A, C)
    is_minimal = (rank_c == n) and (rank_o == n)
    return is_minimal, n


def plot_open_loop_poles(A, save_name='phase1_open_loop_poles'):
    """Plot open-loop poles on the complex plane and save as PDF + PNG."""
    from plot_utils import new_fig, save_fig, finalize, C_UNSTABLE, C_STABLE

    eigs = np.linalg.eigvals(A)
    fig, ax = new_fig(width=5.5, height=4.0)
    ax = fig.axes[0]

    ax.axvline(0, color='k', linewidth=0.7, linestyle='--', alpha=0.5)
    ax.axhline(0, color='k', linewidth=0.7, linestyle='--', alpha=0.5)

    for e in eigs:
        color = C_UNSTABLE if np.real(e) > 1e-9 else C_STABLE
        ax.scatter(np.real(e), np.imag(e), marker='x', s=150,
                   color=color, linewidths=2.5, zorder=5)
        ax.annotate(f'${np.real(e):.3f}$', xy=(np.real(e), np.imag(e)),
                    xytext=(8, 5), textcoords='offset points', fontsize=8.5)

    ax.scatter([], [], marker='x', s=100, color=C_UNSTABLE,
               linewidths=2.5, label='Unstable (RHP)')
    ax.scatter([], [], marker='x', s=100, color=C_STABLE,
               linewidths=2.5, label='Stable / marginal (LHP)')

    ax.set_title('Open-Loop Poles — Two-Wheel Self-Balancing Robot')
    ax.set_xlabel('Real part (rad/s)')
    ax.set_ylabel('Imaginary part (rad/s)')
    ax.legend()
    save_fig(finalize(fig), save_name)
