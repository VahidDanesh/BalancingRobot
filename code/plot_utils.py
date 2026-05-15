"""Publication-quality plot helper for MEC 560 final project.

All figures are saved as PDF (vector) to plots/.
A PNG copy is also saved for quick preview.

Color convention (consistent across all project figures):
    plant states    -> C_PLANT (blue),  solid
    observer states -> C_OBS   (orange), dashed
    reference       -> C_REF   (green),  dashed
    control input   -> C_CTRL  (red),    solid
"""

import os
import matplotlib as mpl
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Directory
# ---------------------------------------------------------------------------
PLOT_DIR = os.path.join(os.path.dirname(__file__), 'plots')

# ---------------------------------------------------------------------------
# Color palette
# ---------------------------------------------------------------------------
C_PLANT = '#1f77b4'   # blue   — true plant states
C_OBS   = '#ff7f0e'   # orange — observer estimates
C_REF   = '#2ca02c'   # green  — reference trajectory
C_CTRL  = '#d62728'   # red    — control input
C_UNSTABLE = '#d62728'
C_STABLE   = '#1f77b4'

# Qualitative palette for multi-curve plots (uncertainty sweep, multi-IC)
PALETTE = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
           '#8c564b', '#e377c2', '#7f7f7f']

# ---------------------------------------------------------------------------
# RC-params style — matches IEEE/ASME paper aesthetics
# ---------------------------------------------------------------------------
_STYLE = {
    # Fonts
    'font.family':        'serif',
    'font.serif':         ['Times New Roman', 'Times', 'DejaVu Serif', 'serif'],
    'mathtext.fontset':   'stix',
    'font.size':          10,
    'axes.titlesize':     10,
    'axes.labelsize':     10,
    'xtick.labelsize':    9,
    'ytick.labelsize':    9,
    'legend.fontsize':    8.5,
    'legend.framealpha':  0.85,
    'legend.edgecolor':   '0.7',
    # Lines
    'lines.linewidth':    1.6,
    'lines.markersize':   5,
    # Axes
    'axes.linewidth':     0.8,
    'axes.spines.top':    False,
    'axes.spines.right':  False,
    # Grid
    'axes.grid':          True,
    'grid.alpha':         0.3,
    'grid.linestyle':     '--',
    'grid.linewidth':     0.5,
    'grid.color':         '0.5',
    # Save
    'figure.dpi':         150,
    'savefig.dpi':        300,
    'savefig.bbox':       'tight',
    'savefig.pad_inches': 0.04,
}


def apply_style():
    """Apply the project-wide rcParams. Call once at the top of a script."""
    mpl.rcParams.update(_STYLE)


def new_fig(nrows=1, ncols=1, width=6.5, height=None, **kwargs):
    """Create a styled figure.

    Parameters
    ----------
    nrows, ncols : int   — subplot grid layout
    width        : float — total figure width in inches (6.5 ≈ IEEE full-width)
    height       : float — total height; defaults to golden-ratio scaling per row
    **kwargs     : passed to plt.subplots

    Returns
    -------
    fig  : Figure
    axes : Axes or ndarray of Axes
    """
    apply_style()
    if height is None:
        height = width * 0.62 * nrows / max(ncols, 1)
    fig, axes = plt.subplots(nrows, ncols, figsize=(width, height), **kwargs)
    return fig, axes


def save_fig(fig, name, pdf=True, png=True):
    """Save figure to plots/ as PDF and/or PNG.

    Parameters
    ----------
    fig  : matplotlib Figure
    name : str  — base filename without extension
    pdf  : bool — save PDF (default True)
    png  : bool — save PNG preview (default True)
    """
    os.makedirs(PLOT_DIR, exist_ok=True)
    if pdf:
        path = os.path.join(PLOT_DIR, name + '.pdf')
        fig.savefig(path)
        # print(f"  Saved: {path}")
    if png:
        path_png = os.path.join(PLOT_DIR, name + '.png')
        fig.savefig(path_png, dpi=150)
    plt.close(fig)


def finalize(fig, tight=True):
    """Apply tight_layout and clean up spines consistently."""
    if tight:
        fig.tight_layout()
    return fig
