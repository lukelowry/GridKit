#!/usr/bin/env python3
"""Plot Huge WECC generator omega monitor output."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError as exc:
    print(f"Missing Python dependency: {exc}", file=sys.stderr)
    print("Install required packages: python3-numpy python3-pandas python3-matplotlib", file=sys.stderr)
    sys.exit(2)


@dataclass(frozen=True)
class Event:
    time: float
    label: str
    linestyle: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("mon.csv"), help="Monitor CSV path")
    parser.add_argument("--solver", type=Path, default=Path("wecc.solver.json"), help="Solver JSON path")
    parser.add_argument("--output", type=Path, default=Path("wecc.generator_omega.png"), help="Output PNG path")
    parser.add_argument("--runtime-seconds", type=float, help="Simulation wall-clock time for the title")
    parser.add_argument("--max-traces", type=int, help="Maximum omega traces to draw")
    parser.add_argument("--show", action="store_true", help="Also show the plot interactively")
    return parser


def load_solver(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def load_events(solver: dict) -> list[Event]:
    labels = {
        "fault_on": ("fault on", "--"),
        "fault_off": ("clear", ":"),
    }

    events: list[Event] = []
    for raw in solver.get("events", []):
        if "time" not in raw:
            continue
        event_type = str(raw.get("type", "event"))
        label, linestyle = labels.get(event_type, (event_type.replace("_", " "), "--"))
        events.append(Event(float(raw["time"]), label, linestyle))
    return events


def clean_monitor_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")

    df = pd.read_csv(path, dtype=str)
    if df.empty:
        raise ValueError(f"CSV has no rows: {path}")

    df.columns = [str(col).strip() for col in df.columns]
    first_col = df.columns[0]
    df = df.loc[df[first_col].astype(str).str.strip() != first_col].copy()

    if "t" not in df.columns:
        if "time" not in df.columns:
            raise ValueError("Expected monitor CSV to contain 't' or 'time'.")
        df = df.rename(columns={"time": "t"})

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["t"])
    df = df.dropna(axis=0, how="all")
    df = df.sort_values("t")
    df = df.drop_duplicates(subset=["t"], keep="last")
    df = df.reset_index(drop=True)
    if df.empty:
        raise ValueError(f"CSV has no numeric rows after cleanup: {path}")
    return df


def omega_columns(df: pd.DataFrame) -> list[str]:
    columns = sorted(col for col in df.columns if col.endswith("_omega"))
    if not columns:
        raise ValueError("No omega monitor columns found.")
    return columns


def selected_trace_indices(count: int, max_traces: int | None) -> list[int]:
    if max_traces is None or count <= max_traces:
        return list(range(count))
    return list(dict.fromkeys(np.linspace(0, count - 1, num=max_traces, dtype=int).tolist()))


def title_for(solver: dict, runtime_seconds: float | None) -> str:
    tmax = solver.get("tmax")
    if isinstance(tmax, (int, float)):
        t_part = f"t=0-{tmax:g} s"
    else:
        t_part = "t range"

    title = f"Huge WECC generator speed deviation, {t_part}"
    if runtime_seconds is not None:
        title += f" | completed in {runtime_seconds:.4f} s"
    return title


def x_limits(df: pd.DataFrame, solver: dict) -> tuple[float, float]:
    tmax = solver.get("tmax")
    if isinstance(tmax, (int, float)):
        return 0.0, float(tmax)
    return 0.0, float(df["t"].max())


def add_events(ax: plt.Axes, events: list[Event]) -> None:
    for index, event in enumerate(events):
        ax.axvline(event.time, color="#334155", linestyle=event.linestyle, linewidth=1.1, alpha=0.95)
        alignment = "right" if index == 0 else "left"
        x_offset = -0.01 if index == 0 else 0.01
        ax.annotate(
            event.label,
            xy=(event.time, 0.985),
            xycoords=("data", "axes fraction"),
            xytext=(x_offset, 0.0),
            textcoords="offset fontsize",
            ha=alignment,
            va="top",
            rotation=90,
            color="#334155",
            fontsize=9,
        )


def style_axis(ax: plt.Axes, solver: dict, events: list[Event]) -> None:
    ax.set_facecolor("#f8fafc")
    ax.grid(True, color="#cbd5e1", linewidth=0.75, alpha=0.6)
    ax.set_axisbelow(True)

    for spine in ax.spines.values():
        spine.set_color("#cbd5e1")
        spine.set_linewidth(0.9)

    ax.tick_params(colors="#64748b", labelsize=10)
    ax.xaxis.label.set_color("#334155")
    ax.yaxis.label.set_color("#334155")

    x0, x1 = x_limits_from_axis(ax)
    if x1 >= 1.0:
        ax.set_xticks(np.arange(1.0, np.floor(x1) + 1.0, 1.0))

    ax.set_xlabel("Time [s]", fontsize=11)
    ax.set_ylabel("Generator omega [p.u.]", fontsize=11)
    ax.set_title(title_for(solver, None), fontsize=16, fontweight="bold", color="#111827", pad=16)
    add_events(ax, events)


def x_limits_from_axis(ax: plt.Axes) -> tuple[float, float]:
    left, right = ax.get_xlim()
    return float(left), float(right)


def plot_omega(
    df: pd.DataFrame,
    solver: dict,
    output: Path,
    runtime_seconds: float | None,
    max_traces: int | None,
    show: bool,
) -> int:
    columns = omega_columns(df)
    indices = selected_trace_indices(len(columns), max_traces)

    fig, ax = plt.subplots(figsize=(18, 7.68))
    fig.patch.set_facecolor("#f1f5f9")

    cmap = plt.cm.viridis
    norm = mpl.colors.Normalize(vmin=0, vmax=max(len(columns) - 1, 1))
    t = df["t"]

    for index in indices:
        ax.plot(
            t,
            df[columns[index]],
            color=cmap(norm(index)),
            linewidth=0.55,
            alpha=0.22,
        )

    ax.set_xlim(*x_limits(df, solver))
    style_axis(ax, solver, load_events(solver))
    ax.set_title(title_for(solver, runtime_seconds), fontsize=16, fontweight="bold", color="#111827", pad=16)

    scalar_map = mpl.cm.ScalarMappable(norm=norm, cmap=cmap)
    scalar_map.set_array([])
    cbar = fig.colorbar(scalar_map, ax=ax, pad=0.018, fraction=0.025)
    cbar.set_label("Generator trace index", color="#334155", fontsize=11)
    cbar.ax.tick_params(colors="#64748b", labelsize=10)
    cbar.outline.set_color("#cbd5e1")

    fig.subplots_adjust(left=0.07, right=0.94, top=0.88, bottom=0.11)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=224)
    if show:
        plt.show()
    plt.close(fig)
    return len(indices)


def main() -> int:
    args = build_parser().parse_args()
    if args.max_traces is not None and args.max_traces < 1:
        raise ValueError("--max-traces must be >= 1")

    solver = load_solver(args.solver)
    df = clean_monitor_csv(args.input)
    plotted = plot_omega(df, solver, args.output, args.runtime_seconds, args.max_traces, args.show)
    print(f"{args.output} ({plotted} omega traces)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
