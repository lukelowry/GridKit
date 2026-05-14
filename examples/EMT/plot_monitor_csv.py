#!/usr/bin/env python3
"""Plot EMT monitor-architecture CSV output."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError as exc:
    print(f"Missing Python dependency: {exc}", file=sys.stderr)
    print(
        "Install required packages: python3-numpy python3-pandas python3-matplotlib",
        file=sys.stderr,
    )
    sys.exit(2)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_INPUT = REPO_ROOT / "build/examples/EMT/Tiny/TwoBus/EMTTinyTwoBus.csv"
DEFAULT_OUTPUT = REPO_ROOT / "build/examples/EMT/Tiny/TwoBus/EMTTinyTwoBus.png"
PHASES = ("a", "b", "c")
PHASE_COLORS = {"a": "#2563eb", "b": "#16a34a", "c": "#dc2626"}
SOURCE_LOAD_STYLES = {
    "source_bus": ("Source bus", "-"),
    "load_bus": ("Load bus", "--"),
    "source": ("Source", "-"),
    "load": ("Load", "--"),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="CSV produced by an EMT monitor-enabled example",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="PNG path to write",
    )
    return parser


def phase_groups(data: pd.DataFrame, prefix: str) -> dict[str, dict[str, str]]:
    groups: dict[str, dict[str, str]] = {}
    wanted = {f"{prefix}{phase}": phase for phase in PHASES}
    for column in data.columns:
        if "_" not in column:
            continue

        group, variable = column.rsplit("_", 1)
        phase = wanted.get(variable)
        if phase is None:
            continue

        groups.setdefault(group, {})[phase] = column

    return {group: cols for group, cols in groups.items() if set(cols) == set(PHASES)}


def fault_active_mask(data: pd.DataFrame, fault_columns: dict[str, str]) -> np.ndarray:
    ordered = [fault_columns[phase] for phase in PHASES]
    magnitude = np.max(np.abs(data[ordered].to_numpy(dtype=float)), axis=1)
    if magnitude.size == 0 or np.max(magnitude) == 0.0:
        return np.zeros(len(data), dtype=bool)

    return magnitude > max(1.0e-9, 1.0e-6 * float(np.max(magnitude)))


def fault_window(data: pd.DataFrame, fault_columns: dict[str, str]) -> tuple[float | None, float | None]:
    active = fault_active_mask(data, fault_columns)
    if not np.any(active):
        return None, None

    transitions = np.diff(active.astype(int))
    starts = np.flatnonzero(transitions > 0)
    clears = np.flatnonzero(transitions < 0)

    start_time = float(data["t"].iloc[starts[0] + 1]) if starts.size else None
    clear_time = float(data["t"].iloc[clears[0] + 1]) if clears.size else None
    return start_time, clear_time


def pretty(label: str) -> str:
    return label.replace("_", " ")


def display_name(group: str, labels: dict[str, str] | None = None) -> str:
    if labels and group in labels:
        return labels[group]
    return SOURCE_LOAD_STYLES.get(group, (pretty(group).title(), "-"))[0]


def line_style(group: str, fallback_index: int = 0) -> str:
    fallback_styles = ("-", "--", ":", "-.")
    return SOURCE_LOAD_STYLES.get(group, ("", fallback_styles[fallback_index % len(fallback_styles)]))[1]


def ordered_groups(groups: dict[str, dict[str, str]], preferred: tuple[str, ...]) -> dict[str, dict[str, str]]:
    ordered = {group: groups[group] for group in preferred if group in groups}
    ordered.update({group: cols for group, cols in groups.items() if group not in ordered})
    return ordered


def source_load_pair(groups: dict[str, dict[str, str]], preferred: tuple[str, str]) -> dict[str, dict[str, str]]:
    ordered = ordered_groups(groups, preferred)
    return dict(list(ordered.items())[:2])


def add_fault_window(ax: plt.Axes, start_time_ms: float | None, clear_time_ms: float | None) -> None:
    if start_time_ms is not None and clear_time_ms is not None:
        ax.axvspan(start_time_ms, clear_time_ms, color="#64748b", alpha=0.12, linewidth=0)
    if start_time_ms is not None:
        ax.axvline(start_time_ms, color="#64748b", linestyle="--", linewidth=1.0)
    if clear_time_ms is not None:
        ax.axvline(clear_time_ms, color="#334155", linestyle=":", linewidth=1.1)


def style_axis(ax: plt.Axes) -> None:
    ax.grid(True, color="#cbd5e1", alpha=0.55, linewidth=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#94a3b8")
    ax.spines["bottom"].set_color("#94a3b8")
    ax.tick_params(colors="#334155", labelsize=9)


def add_phase_location_legend(
    ax: plt.Axes,
    groups: dict[str, dict[str, str]],
    labels: dict[str, str] | None = None,
) -> None:
    from matplotlib.lines import Line2D

    phase_handles = [
        Line2D([0], [0], color=PHASE_COLORS[phase], linewidth=2.0, label=f"Phase {phase.upper()}")
        for phase in PHASES
    ]
    location_handles = [
        Line2D(
            [0],
            [0],
            color="#111827",
            linestyle=line_style(group, index),
            linewidth=2.0,
            label=display_name(group, labels),
        )
        for index, group in enumerate(groups)
    ]

    first = ax.legend(handles=phase_handles, loc="upper right", ncol=3, fontsize=8, frameon=False)
    ax.add_artist(first)
    ax.legend(handles=location_handles, loc="upper left", ncol=max(1, len(location_handles)), fontsize=8, frameon=False)


def plot_phase_pair(
    ax: plt.Axes,
    t_ms: pd.Series,
    data: pd.DataFrame,
    groups: dict[str, dict[str, str]],
    title: str,
    ylabel: str,
    scales: dict[str, float] | None = None,
    labels: dict[str, str] | None = None,
) -> None:
    for group_index, (group, columns) in enumerate(groups.items()):
        style = line_style(group, group_index)
        width = 1.55 if style == "-" else 1.35
        scale = scales.get(group, 1.0) if scales else 1.0
        for phase in PHASES:
            ax.plot(
                t_ms,
                scale * data[columns[phase]],
                color=PHASE_COLORS[phase],
                linestyle=style,
                linewidth=width,
                alpha=0.92,
            )
    ax.set_title(title, loc="left", fontsize=11, fontweight="semibold")
    ax.set_ylabel(ylabel)
    add_phase_location_legend(ax, groups, labels)


def main() -> int:
    args = build_parser().parse_args()
    csv_path = args.input.resolve()
    output_path = args.output.resolve()

    if not csv_path.exists():
        print(f"CSV not found: {csv_path}", file=sys.stderr)
        print("Run: cmake --build build --target EMTTinyTwoBus -j2", file=sys.stderr)
        print("Then: build/examples/EMT/Tiny/TwoBus/EMTTinyTwoBus", file=sys.stderr)
        return 1

    data = pd.read_csv(csv_path)
    if "t" not in data:
        print(f"CSV has no time column named 't': {csv_path}", file=sys.stderr)
        return 1

    voltage_groups = source_load_pair(phase_groups(data, "v"), ("source_bus", "load_bus"))
    current_groups = source_load_pair(phase_groups(data, "i"), ("source", "load"))
    fault_groups = phase_groups(data, "if")

    if not voltage_groups and not current_groups:
        print(f"No EMT phase monitor columns found in {csv_path}", file=sys.stderr)
        return 1

    fault_start, fault_clear = (None, None)
    if fault_groups:
        fault_start, fault_clear = fault_window(data, next(iter(fault_groups.values())))

    t_ms = data["t"] * 1000.0
    fault_start_ms = fault_start * 1000.0 if fault_start is not None else None
    fault_clear_ms = fault_clear * 1000.0 if fault_clear is not None else None

    fig, axes = plt.subplots(2, 1, figsize=(13.5, 6.4), sharex=True, constrained_layout=True)

    plot_phase_pair(axes[0], t_ms, data, voltage_groups, "Bus Phase Voltages", "voltage [V]")
    plot_phase_pair(
        axes[1],
        t_ms,
        data,
        current_groups,
        "Source and Load Phase Currents",
        "current [A]",
        scales={"load": -1.0},
        labels={"load": "-Load"},
    )
    axes[1].set_xlabel("time [ms]")

    for ax in axes:
        style_axis(ax)
        add_fault_window(ax, fault_start_ms, fault_clear_ms)

    if fault_start_ms is not None and fault_clear_ms is not None:
        axes[0].annotate(
            "fault",
            xy=((fault_start_ms + fault_clear_ms) * 0.5, 0.96),
            xycoords=("data", "axes fraction"),
            ha="center",
            va="top",
            fontsize=8,
            color="#334155",
        )

    fig.suptitle(f"EMT TwoBus Monitor: {csv_path.stem}", fontsize=14, fontweight="semibold")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
