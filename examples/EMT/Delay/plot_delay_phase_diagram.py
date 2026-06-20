#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch, Polygon


PHASE_METHOD = "ArkStep forward Euler phase-map fixed h"

LEFT_SPECS = [
    {"case": "phase_out_of_band_stable", "title": "High band", "color": "#7b519d"},
    {"case": "phase_boundary_in_band", "title": "Mid band", "color": "#b47a13"},
    {"case": "phase_good_damped", "title": "Low band", "color": "#2f7f62"},
]

RIGHT_SPECS = [
    {"case": "phase_good_near_band", "title": "High band", "color": "#2f6f9f"},
    {"case": "phase_good_coarse_n1", "title": "Mid band", "color": "#7a7f2f"},
    {"case": "phase_good_sample_shift", "title": "Low band", "color": "#5f6c6c"},
]

VISIBLE_STEPS = 10.0


def parse_float(value):
    return float(value) if value else None


def read_rows(path):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                {
                    "case": row["case"],
                    "tau": parse_float(row["tau"]),
                    "fmax": parse_float(row["fmax"]),
                    "frequency": parse_float(row.get("frequency", "")),
                    "N": int(row["N"]),
                    "T": parse_float(row["T"]),
                    "method": row["method"],
                    "h": parse_float(row["h"]),
                    "h_over_T": parse_float(row["h_over_T"]),
                    "h_over_tau": parse_float(row["h_over_tau"]),
                    "t": parse_float(row["t"]),
                    "ideal_delay": parse_float(row["ideal_delay"]),
                    "output": parse_float(row["output"]),
                    "error": parse_float(row["error"]),
                }
            )
    return rows


def rows_for_case(rows, case):
    selected = [
        row
        for row in rows
        if row["case"] == case
        and row["method"] == PHASE_METHOD
        and row["output"] is not None
        and row["ideal_delay"] is not None
        and row["t"] > 0.0
    ]
    return sorted(selected, key=lambda row: row["t"])


def time_response_points(t, output, tau, h):
    t_end = float(t.max())
    t_start = max(float(t.min()), tau, t_end - VISIBLE_STEPS * h)
    mask = (t >= t_start - 1.0e-12) & (t <= t_end + 1.0e-12)
    return t[mask], output[mask], t_start, t_end


def plot_map(ax):
    x_min = 2.0 ** -3.55
    x_max = 1.0
    y_min = 2.0 ** -3.35
    y_max = 1.0

    ax.axhspan(y_min, y_max, color="#bddccb", alpha=0.88, linewidth=0.0)

    ax.set_box_aspect(1.0)
    ax.set_anchor("C")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xscale("log", base=2.0)
    ax.set_yscale("log", base=2.0)
    ax.set_xticks([2.0 ** -3, 2.0 ** -2, 2.0 ** -1, 2.0 ** 0])
    ax.set_xticklabels([r"$2^{-3}$", r"$2^{-2}$", r"$2^{-1}$", r"$2^{0}$"])
    ax.set_yticks([2.0 ** -3, 2.0 ** -2, 2.0 ** -1, 2.0 ** 0])
    ax.set_yticklabels([r"$2^{-3}$", r"$2^{-2}$", r"$2^{-1}$", r"$2^{0}$"])
    ax.minorticks_off()
    ax.set_xlabel(r"$\dfrac{h}{\tau}$", fontsize=21)
    ax.set_ylabel(r"$\dfrac{2f}{f_{\max}}$", fontsize=21)
    ax.set_title(r"Representative Delay/FE samples with $h=T$", fontsize=16, pad=14)
    algebraic_region = Polygon(
        [(0.50, 1.0), (1.0, 1.0), (1.0, 0.50)],
        closed=True,
        facecolor="#111111",
        edgecolor="none",
        alpha=0.94,
        transform=ax.transAxes,
        zorder=3,
    )
    ax.add_patch(algebraic_region)
    ax.text(
        0.61,
        0.63,
        "most efficient",
        ha="center",
        va="center",
        fontsize=13,
        color="#26332e",
        rotation=-45,
        rotation_mode="anchor",
        transform=ax.transAxes,
    )
    ax.text(
        0.25,
        0.50,
        "High RAM",
        ha="center",
        va="center",
        rotation=90,
        fontsize=13,
        color="#26332e",
        transform=ax.transAxes,
    )
    ax.text(
        0.75,
        0.16,
        "time step should increase",
        ha="center",
        va="center",
        fontsize=12,
        color="#26332e",
        transform=ax.transAxes,
        zorder=4,
    )
    ax.grid(True, which="major", color="#6f8178", alpha=0.32, linewidth=0.65)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color("#333333")
        spine.set_linewidth(0.9)


def plot_response(ax, rows, spec):
    case_rows = rows_for_case(rows, spec["case"])
    if not case_rows:
        raise RuntimeError(f"Could not find phase-diagram rows for {spec['case']}.")

    t = np.array([row["t"] for row in case_rows])
    output = np.array([row["output"] for row in case_rows])
    tau = case_rows[0]["tau"]
    section_count = case_rows[0]["N"]
    fmax = case_rows[0]["fmax"]
    frequency = case_rows[0]["frequency"]
    if frequency is None:
        raise RuntimeError(f"Missing frequency column for phase-diagram case {spec['case']}.")
    h_over_tau = case_rows[0]["h_over_tau"]
    h = case_rows[0]["h"]
    f_ratio = 2.0 * frequency / fmax
    t_plot, output_plot, t_start, t_end = time_response_points(t, output, tau, h)
    t_ideal = np.linspace(t_start, t_end, 1200)
    ideal = np.sin(2.0 * np.pi * frequency * (t_ideal - tau))

    ax.plot(t_ideal, ideal, color="#575757", linestyle=(0, (4, 2)), linewidth=1.0)
    ax.plot(
        t_plot,
        output_plot,
        color=spec["color"],
        linewidth=1.25,
        marker="o",
        markersize=2.0,
        markeredgewidth=0.0,
    )
    ax.axhline(0.0, color="#999999", linewidth=0.55, alpha=0.58)
    ax.set_xlim(t_start, t_end)
    ax.set_ylabel(r"$y(t)$")
    ax.set_title(rf"{spec['title']}  $N={section_count}$", loc="left", fontsize=10.5, pad=4)
    ax.grid(False)
    ax.tick_params(axis="both", labelsize=9.5)
    if np.max(np.abs(output_plot)) > 100.0:
        ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
    for spine in ax.spines.values():
        spine.set_linewidth(0.82)
        spine.set_color("#333333")

    return {
        "axis": ax,
        "x": h_over_tau,
        "y": f_ratio,
        "color": spec["color"],
        "anchor_y": spec.get("anchor_y", 0.50),
    }


def add_leader_lines(fig, map_ax, targets):
    for target in targets:
        map_ax.plot(
            target["x"],
            target["y"],
            marker="o",
            markersize=6.0,
            markerfacecolor=target["color"],
            markeredgecolor="white",
            markeredgewidth=1.1,
            zorder=8,
        )
        panel_x = 1.0 if target["side"] == "left" else 0.0
        connector = ConnectionPatch(
            xyA=(target["x"], target["y"]),
            coordsA=map_ax.transData,
            xyB=(panel_x, target["anchor_y"]),
            coordsB=target["axis"].transAxes,
            color=target["color"],
            linewidth=1.0,
            alpha=0.70,
            zorder=2,
        )
        connector.set_clip_on(False)
        fig.add_artist(connector)


def main():
    if len(sys.argv) > 2:
        csv_path = Path(sys.argv[1])
        png_path = Path(sys.argv[2])
    elif len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
        png_path = Path("delay_phase_diagram.png")
    else:
        csv_path = Path("delay_response.csv")
        png_path = Path("delay_phase_diagram.png")

    rows = read_rows(csv_path)
    png_path.parent.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Liberation Serif", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "axes.unicode_minus": False,
        "font.size": 11,
        "axes.labelsize": 12,
        "xtick.labelsize": 10.5,
        "ytick.labelsize": 10.5,
    })

    fig = plt.figure(figsize=(17.2, 8.4), facecolor="white")
    grid = fig.add_gridspec(
        3,
        3,
        width_ratios=[1.04, 1.35, 1.04],
        wspace=0.28,
        hspace=0.70,
        left=0.045,
        right=0.985,
        top=0.91,
        bottom=0.090,
    )

    left_axes = [fig.add_subplot(grid[i, 0]) for i in range(3)]
    map_ax = fig.add_subplot(grid[:, 1])
    right_axes = [fig.add_subplot(grid[i, 2]) for i in range(3)]

    plot_map(map_ax)

    targets = []
    for axis, spec in zip(left_axes, LEFT_SPECS):
        target = plot_response(axis, rows, spec)
        target["side"] = "left"
        targets.append(target)
    for axis, spec in zip(right_axes, RIGHT_SPECS):
        target = plot_response(axis, rows, spec)
        target["side"] = "right"
        targets.append(target)

    left_axes[-1].set_xlabel(r"$t$ [s]")
    right_axes[-1].set_xlabel(r"$t$ [s]")
    for axis in left_axes[:-1] + right_axes[:-1]:
        axis.tick_params(labelbottom=False)

    add_leader_lines(fig, map_ax, targets)
    fig.savefig(png_path, dpi=180)


if __name__ == "__main__":
    main()
