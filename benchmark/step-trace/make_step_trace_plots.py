#!/usr/bin/env python3
"""Trace the IDA step size through a bus fault and plot it against time.

Produces two figures:

  step_size_by_case.png       h(t) for every case at a fixed tolerance
  step_size_by_tolerance.png  h(t) for one case across an integration
                              tolerance sweep

Each run writes a per-step CSV via the solver file's `step_trace_file` key.
The application drives IDA in IDA_ONE_STEP over the same monitor targets a
normal run walks, so the traced step sequence matches an untraced run.

    python3 make_step_trace_plots.py              # prepare, run, plot
    python3 make_step_trace_plots.py --plot-only  # replot existing traces
    python3 make_step_trace_plots.py --cases wecc texas
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
EXAMPLES = ROOT / "examples" / "PhasorDynamics"
BIN = ROOT / "build" / "application" / "PhasorDynamics" / "DynamicSimulation"
WORK = HERE / "work"

# Study window. The fault lands early so the recovery occupies most of the plot.
TMAX = 30.0
FAULT_ON = 1.0
FAULT_OFF = 1.1
DT_MONITOR = 1.0 / 240.0

# Nominal system frequency; 1/(4f) is drawn as the step-size reference line.
F_NOMINAL = 60.0

# Each re-initialization restarts IDA at a near-zero step. Clip it away so the
# decade the solver actually works in fills the axis.
YLIM = (1.0e-3, 1.0e0)

# Tolerance used by the per-case figure, and the ratio held between the
# absolute and relative tolerances across the sweep.
BASE_REL_TOL = 1.0e-7
ABS_OVER_REL = 1.0e-2

SWEEP_CASE = "texas"
SWEEP_REL_TOLS = [1.0e-3, 1.0e-5, 1.0e-7, 1.0e-9, 1.0e-11]

# Bus ids come from the case-sweep benchmark variants. A case with no entry
# faults the bus its own BusFault device names.
CASES = {
    "hawaii": dict(case=EXAMPLES / "Medium/Hawaii/hawaii.case.json", fault_bus=1, label="Hawaii"),
    "newengland": dict(case=EXAMPLES / "Medium/NewEngland/newengland.case.json", fault_bus=2, label="NE"),
    "illinois": dict(case=EXAMPLES / "Large/Illinois/illinois.case.json", fault_bus=2, label="Illinois"),
    "wecc": dict(case=EXAMPLES / "Large/WECC/wecc.case.json", fault_bus=3903, label="WECC"),
    "texas": dict(case=EXAMPLES / "Large/Texas/texas.case.json", fault_bus=1027, label="Texas"),
    "activsg10k": dict(case=EXAMPLES / "Huge/activsg10k/ACTIVSg10k.case.json", fault_bus=None, label="ACTIVSg10k"),
    "activsg25k": dict(case=EXAMPLES / "Huge/activsg25k/ACTIVSg25k.case.json", fault_bus=None, label="ACTIVSg25k"),
}


def strip_monitors(node) -> None:
    """Drop every `mon` key so a traced run writes no state output."""
    if isinstance(node, dict):
        node.pop("mon", None)
        for value in node.values():
            strip_monitors(value)
    elif isinstance(node, list):
        for value in node:
            strip_monitors(value)


def prepare_case(name: str) -> Path:
    """Write a monitor-free copy of a case, reusing it if already current."""
    src = CASES[name]["case"]
    dst = WORK / f"{name}.case.json"
    if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
        return dst
    if not src.exists():
        raise FileNotFoundError(f"case file missing: {src}")
    case = json.loads(src.read_text())
    strip_monitors(case)
    case["monitors"] = []
    dst.write_text(json.dumps(case))
    return dst


def write_solver(tag: str, name: str, rel_tol: float) -> tuple[Path, Path]:
    case_path = prepare_case(name)
    trace_path = WORK / f"{tag}.trace.csv"
    study = {
        "system_model_file": case_path.name,
        "dt_monitor": DT_MONITOR,
        "tmax": TMAX,
        "events": [
            {"time": FAULT_ON, "type": "fault_on"},
            {"time": FAULT_OFF, "type": "fault_off"},
        ],
        "step_trace_file": trace_path.name,
        "ida": {
            "rel_tol": rel_tol,
            "abs_tol": rel_tol * ABS_OVER_REL,
            "max_num_steps": 10_000_000,
            "line_search_off_ic": True,
        },
    }
    fault_bus = CASES[name]["fault_bus"]
    if fault_bus is not None:
        study["fault_bus"] = fault_bus

    solver_path = WORK / f"{tag}.solver.json"
    solver_path.write_text(json.dumps(study, indent=2))
    return solver_path, trace_path


def run(tag: str, name: str, rel_tol: float, force: bool) -> Path | None:
    solver_path, trace_path = write_solver(tag, name, rel_tol)
    if trace_path.exists() and not force:
        print(f"  {tag}: reusing {trace_path.name}")
        return trace_path

    print(f"  {tag}: running", flush=True)
    result = subprocess.run(
        [str(BIN), solver_path.name],
        cwd=WORK,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0 or not trace_path.exists():
        tail = (result.stderr or result.stdout).strip().splitlines()[-3:]
        print(f"  {tag}: FAILED rc={result.returncode}: {' | '.join(tail)}", file=sys.stderr)
        return None
    print(f"  {tag}: {sum(1 for _ in trace_path.open()) - 1} steps")
    return trace_path


def load(trace_path: Path) -> pd.DataFrame:
    return pd.read_csv(trace_path)


def draw_reference(ax) -> None:
    y = 1.0 / (4.0 * F_NOMINAL)
    ax.axhline(y, color="0.45", linestyle="--", linewidth=0.8, zorder=1)
    ax.annotate(
        r"$\frac{1}{4f}$",
        xy=(1.005, y),
        xycoords=("axes fraction", "data"),
        va="center",
        fontsize=11,
        annotation_clip=False,
    )


def style_axes(ax) -> None:
    ax.set_yscale("log")
    ax.set_xlim(0.0, TMAX)
    ax.set_ylim(*YLIM)
    ax.set_xlabel(r"$t$ $-$ Time [sec]")
    ax.set_ylabel(r"$h$ $-$ Time step [sec]")
    ax.axvline(FAULT_ON, color="0.25", linestyle=":", linewidth=0.9, zorder=1)
    ax.grid(True, which="major", linewidth=0.4, alpha=0.35)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def apply_style() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["DejaVu Serif", "serif"],
        "font.size": 9,
        "axes.labelsize": 10,
        "axes.linewidth": 0.7,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5,
        "legend.frameon": False,
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "lines.linewidth": 1.0,
    })


def plot_by_case(traces: dict[str, Path]) -> Path:
    """h(t) for each case, one line per case, at the base tolerance."""
    apply_style()
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    for i, (name, path) in enumerate(traces.items()):
        frame = load(path)
        ax.step(frame["t"], frame["h"], where="post",
                color=colors[i % 10], label=CASES[name]["label"], linewidth=1.0)

    style_axes(ax)
    draw_reference(ax)
    ncol = min(len(traces), 4)
    rows = -(-len(traces) // ncol)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.08 + 0.08 * rows), ncol=ncol,
              handlelength=1.6, columnspacing=1.4)
    out = HERE / "step_size_by_case.png"
    fig.savefig(out)
    plt.close(fig)
    return out


def plot_by_tolerance(traces: dict[float, Path]) -> Path:
    """h(t) for one case across the tolerance sweep, with a colorbar key."""
    apply_style()
    fig, ax = plt.subplots(figsize=(6.4, 3.4))

    # Keyed on -log10(rel_tol) so the loosest tolerance sits at the left of
    # the bar and the sweep reads left to right as it tightens.
    keys = [-np.log10(r) for r in traces]
    cmap = plt.cm.turbo
    norm = matplotlib.colors.Normalize(vmin=min(keys), vmax=max(keys))

    for rel_tol, path in sorted(traces.items(), reverse=True):
        frame = load(path)
        ax.step(frame["t"], frame["h"], where="post",
                color=cmap(norm(-np.log10(rel_tol))), linewidth=1.0)

    style_axes(ax)
    draw_reference(ax)

    bar = fig.colorbar(
        plt.cm.ScalarMappable(norm=norm, cmap=cmap),
        ax=ax, orientation="horizontal", location="top",
        fraction=0.055, pad=0.04, aspect=40,
    )
    bar.set_ticks(sorted(keys))
    bar.set_ticklabels([rf"$10^{{{-int(k)}}}$" for k in sorted(keys)])
    bar.set_label(r"$r_{tol}$", labelpad=4)
    bar.ax.tick_params(labelsize=8.5, length=0)
    bar.outline.set_linewidth(0.5)

    out = HERE / "step_size_by_tolerance.png"
    fig.savefig(out)
    plt.close(fig)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", default=list(CASES),
                        help="cases for the per-case figure")
    parser.add_argument("--sweep-case", default=SWEEP_CASE,
                        help="case used for the tolerance figure")
    parser.add_argument("--plot-only", action="store_true",
                        help="skip running, plot the traces already in work/")
    parser.add_argument("--force", action="store_true",
                        help="re-run even when a trace already exists")
    parser.add_argument("--skip-sweep", action="store_true",
                        help="build only the per-case figure")
    args = parser.parse_args()

    unknown = [c for c in args.cases if c not in CASES]
    if unknown:
        parser.error(f"unknown case(s): {', '.join(unknown)}")
    if not BIN.exists() and not args.plot_only:
        parser.error(f"application not built: {BIN}")

    WORK.mkdir(parents=True, exist_ok=True)

    case_traces: dict[str, Path] = {}
    print("Per-case traces:")
    for name in args.cases:
        tag = f"case-{name}"
        path = WORK / f"{tag}.trace.csv"
        if args.plot_only:
            if path.exists():
                case_traces[name] = path
            else:
                print(f"  {tag}: no trace, skipped")
            continue
        result = run(tag, name, BASE_REL_TOL, args.force)
        if result is not None:
            case_traces[name] = result

    tol_traces: dict[float, Path] = {}
    if not args.skip_sweep:
        print(f"Tolerance sweep on {args.sweep_case}:")
        for rel_tol in SWEEP_REL_TOLS:
            tag = f"tol-{args.sweep_case}-{rel_tol:.0e}"
            path = WORK / f"{tag}.trace.csv"
            if args.plot_only:
                if path.exists():
                    tol_traces[rel_tol] = path
                else:
                    print(f"  {tag}: no trace, skipped")
                continue
            result = run(tag, args.sweep_case, rel_tol, args.force)
            if result is not None:
                tol_traces[rel_tol] = result

    if case_traces:
        print("wrote", plot_by_case(case_traces))
    if tol_traces:
        print("wrote", plot_by_tolerance(tol_traces))
    if not case_traces and not tol_traces:
        print("no traces to plot", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
