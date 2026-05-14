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
GROUP_STYLES = ("-", "--", ":", "-.")


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


def rms(data: pd.DataFrame, columns: dict[str, str]) -> np.ndarray:
    ordered = [columns[phase] for phase in PHASES]
    return np.sqrt(np.mean(data[ordered].to_numpy(dtype=float) ** 2, axis=1))


def fault_window(data: pd.DataFrame, fault_columns: dict[str, str]) -> tuple[float | None, float | None]:
    ordered = [fault_columns[phase] for phase in PHASES]
    magnitude = np.max(np.abs(data[ordered].to_numpy(dtype=float)), axis=1)
    if magnitude.size == 0 or np.max(magnitude) == 0.0:
        return None, None

    active = magnitude > max(1.0e-9, 1.0e-6 * float(np.max(magnitude)))
    transitions = np.diff(active.astype(int))
    starts = np.flatnonzero(transitions > 0)
    clears = np.flatnonzero(transitions < 0)

    start_time = float(data["t"].iloc[starts[0] + 1]) if starts.size else None
    clear_time = float(data["t"].iloc[clears[0] + 1]) if clears.size else None
    return start_time, clear_time


def pretty(label: str) -> str:
    return label.replace("_", " ")


def plot_phase_groups(ax: plt.Axes, t: pd.Series, data: pd.DataFrame, groups: dict[str, dict[str, str]]) -> None:
    for group_index, (group, columns) in enumerate(groups.items()):
        style = GROUP_STYLES[group_index % len(GROUP_STYLES)]
        for phase in PHASES:
            ax.plot(
                t,
                data[columns[phase]],
                color=PHASE_COLORS[phase],
                linestyle=style,
                linewidth=1.35,
                label=f"{pretty(group)} {phase.upper()}",
            )


def mark_fault_window(axes: list[plt.Axes], start_time: float | None, clear_time: float | None) -> None:
    for ax in axes:
        if start_time is not None:
            ax.axvline(start_time, color="#6b7280", linestyle="--", linewidth=1.1)
        if clear_time is not None:
            ax.axvline(clear_time, color="#111827", linestyle=":", linewidth=1.2)


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

    voltage_groups = phase_groups(data, "v")
    current_groups = phase_groups(data, "i")
    fault_groups = {group: cols for group, cols in current_groups.items() if "fault" in group.lower()}
    circuit_current_groups = {
        group: cols for group, cols in current_groups.items() if group not in fault_groups
    }

    if not voltage_groups and not current_groups:
        print(f"No EMT phase monitor columns found in {csv_path}", file=sys.stderr)
        return 1

    fault_start, fault_clear = (None, None)
    if fault_groups:
        fault_start, fault_clear = fault_window(data, next(iter(fault_groups.values())))

    t = data["t"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    plot_phase_groups(axes[0], t, data, voltage_groups)
    axes[0].set_ylabel("bus voltage [V]")

    for group, columns in voltage_groups.items():
        axes[1].plot(t, rms(data, columns), linewidth=1.6, label=f"{pretty(group)} RMS")
    axes[1].set_ylabel("voltage RMS [V]")

    plot_phase_groups(axes[2], t, data, circuit_current_groups)
    axes[2].set_ylabel("circuit current [A]")

    plot_phase_groups(axes[3], t, data, fault_groups)
    axes[3].set_ylabel("fault current [A]")
    axes[3].set_xlabel("time [s]")

    mark_fault_window(list(axes), fault_start, fault_clear)

    for ax in axes:
        ax.grid(True, alpha=0.25)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="upper right", ncol=3, fontsize=8)

    fig.suptitle(f"EMT Monitor CSV: {csv_path.stem}", y=0.995)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
