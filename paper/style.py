"""Publication matplotlib styling shared by the paper figures."""

from __future__ import annotations

import math
import os
import sys


METRIC_COLORS = ("#3B6EA5", "#2E8B7F", "#7E5AA2", "#C56B2C")
CASE_COLORS = ("#3B6EA5", "#2E8B7F", "#7E5AA2", "#C56B2C", "#B0413E")

LINE_COLOR = "#1A1A1A"
GRID_COLOR = "#E4E8EE"
SPINE_COLOR = "#222222"

QUARTER_CYCLE_SECONDS = 1.0 / (60.0 * 4.0)
DPI = 600

RC_PARAMS = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "STIXGeneral", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8.5,
    "axes.labelsize": 8.5,
    "axes.linewidth": 0.6,
    "axes.edgecolor": SPINE_COLOR,
    "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.major.size": 3.0,
    "ytick.major.size": 3.0,
    "xtick.minor.size": 1.7,
    "ytick.minor.size": 1.7,
    "lines.linewidth": 1.05,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "svg.fonttype": "none",
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
}


def configure(show: bool):
    """Apply publication rcParams and return matplotlib.pyplot."""
    try:
        import matplotlib
    except ImportError as exc:
        raise SystemExit("Plotting requires matplotlib. Install python3-matplotlib.") from exc

    use_pgf = os.environ.get("PAPER_PGF") == "1"
    if "matplotlib.pyplot" not in sys.modules:
        if use_pgf:
            matplotlib.use("pgf")
        elif not show:
            matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    plt.rcParams.update(RC_PARAMS)
    if use_pgf:
        plt.rcParams.update({
            "pgf.texsystem": "pdflatex",
            "text.usetex": True,
            "pgf.rcfonts": False,
            "pgf.preamble": r"\usepackage{newtxtext}\usepackage{newtxmath}",
        })
    return plt


def style_spines(ax) -> None:
    """Apply consistent spine width/color and two-sided ticks."""
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)
        spine.set_color(SPINE_COLOR)
    ax.tick_params(which="both", top=True, right=True)


def iqr_band(ax, data, color) -> float:
    """Shade the interquartile range behind a histogram and return its median."""
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit("Histogram styling requires numpy.") from exc

    q1, median, q3 = (float(v) for v in np.percentile(data, [25, 50, 75]))
    ax.axvspan(q1, q3, color=color, alpha=0.12, linewidth=0, zorder=0)
    return median


def panel_label(ax, letter: str) -> None:
    """Draw a bold panel label in the top-left corner."""
    ax.text(
        0.025,
        0.95,
        f"({letter})",
        transform=ax.transAxes,
        fontsize=8.5,
        fontweight="bold",
        va="top",
        ha="left",
    )


def add_quarter_cycle_reference(ax) -> None:
    """Draw the dashed 1/(4f) quarter-cycle reference line and label."""
    ax.axhline(
        QUARTER_CYCLE_SECONDS,
        color=LINE_COLOR,
        linestyle="--",
        linewidth=0.68,
        alpha=0.62,
        zorder=0,
    )
    ax.text(
        1.012,
        QUARTER_CYCLE_SECONDS,
        r"$\dfrac{1}{4f}$",
        transform=ax.get_yaxis_transform(),
        color=LINE_COLOR,
        fontsize=7.5,
        va="center",
        ha="left",
        clip_on=False,
    )


def log_upper(values, y_min):
    """Round the largest positive value up to the next power of ten."""
    finite = [v for v in values if math.isfinite(v) and v > 0.0]
    if not finite:
        return None
    upper = max(finite) * 1.15
    if y_min is not None:
        upper = max(upper, y_min * 10.0)
    return 10.0 ** math.ceil(math.log10(upper))
