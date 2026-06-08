#!/usr/bin/env python3
"""Lean PhasorDynamics validation runner and plotting CLI."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from cases import by_keys
from plots import (
    plot_contingency,
    plot_contingency_frontier,
    plot_cumulative_work,
    plot_mu,
    plot_signals,
    plot_step_size,
    plot_tolerance,
    plot_tolerance_step_size,
    plot_work_per_step,
)
from studies import (
    DEFAULT_CONTINGENCY_ANALYSIS,
    DEFAULT_DYNAMIC_SIMULATION,
    DEFAULT_OUTPUT_ROOT,
    generate_fault_sweep,
    log_spaced,
    run_binary,
    run_contingency_case,
    run_dynamic_case,
    run_mu_sweep,
    run_tolerance_sweep,
)
from tables import write_error_table


def fault_window_arg(values: list[float] | None) -> tuple[float, float] | None:
    if values is None:
        return None
    if len(values) != 2:
        raise SystemExit("--fault-window requires exactly two values: ON OFF")
    on, off = values
    if off <= on:
        raise SystemExit("--fault-window OFF must be greater than ON")
    return on, off


def add_common_case_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--cases", nargs="*", default=[], help="Case keys. Default: all validation cases.")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT, help="Generated data root.")
    parser.add_argument("--tmax", type=float, default=None, help="Override study tmax.")
    parser.add_argument("--mu", type=float, default=None, help="Override study CommonMath smoothing scale.")
    parser.add_argument("--rel-tol", type=float, default=None, help="Override IDA relative tolerance.")
    parser.add_argument("--abs-tol", type=float, default=None, help="Override IDA absolute tolerance.")
    parser.add_argument("--ida-max-steps", type=int, default=None, help="Override IDA maximum internal steps per solve call.")
    parser.add_argument("--fault-window", nargs=2, type=float, metavar=("ON", "OFF"),
                        help="Move fault_on/fault_off events to a shared window.")


def command_run(args: argparse.Namespace) -> int:
    cases = by_keys(args.cases)
    do_steps = args.steps or not (args.steps or args.contingency)
    do_contingency = args.contingency or not (args.steps or args.contingency)
    fault_window = fault_window_arg(args.fault_window)
    status = 0
    for case in cases:
        print(f"[{case.key}] {case.label}")
        if do_steps:
            result = run_dynamic_case(
                case,
                args.dynamic_simulation,
                args.output_root,
                record_steps=True,
                record_stats=True,
                output_csv=args.output_csv,
                tmax=args.tmax,
                mu=args.mu,
                rel_tol=args.rel_tol,
                abs_tol=args.abs_tol,
                ida_max_steps=args.ida_max_steps,
                fault_window=fault_window,
            )
            status = max(status, result.returncode)
        if do_contingency:
            should_run = case.sweep_by_default or args.all_sweeps or bool(args.cases)
            if not should_run:
                print("  skipping contingency sweep; pass --all-sweeps or select the case explicitly")
                continue
            result = run_contingency_case(
                case,
                args.contingency_analysis,
                args.output_root,
                tmax=args.tmax,
                mu=args.mu,
                rel_tol=args.rel_tol,
                abs_tol=args.abs_tol,
                ida_max_steps=args.ida_max_steps,
                fault_window=fault_window,
            )
            status = max(status, result.returncode)
    return status


def command_tol_sweep(args: argparse.Namespace) -> int:
    rtols = [10.0 ** -exp for exp in range(3, 12)] if args.all_decades else args.rtols
    fault_window = fault_window_arg(args.fault_window)
    for case in by_keys(args.cases):
        run_tolerance_sweep(
            case,
            args.dynamic_simulation,
            args.output_root,
            rtols=rtols,
            atol_ratio=args.atol_ratio,
            repeats=args.repeats,
            tmax=args.tmax,
            mu=args.mu,
            ida_max_steps=args.ida_max_steps,
            output_csv=not args.no_output_csv,
            fault_window=fault_window,
        )
    return 0


def command_mu_sweep(args: argparse.Namespace) -> int:
    mu_values = args.mu_values if args.mu_values else log_spaced(args.mu_min, args.mu_max, args.mu_count)
    fault_window = fault_window_arg(args.fault_window)
    for case in by_keys(args.cases):
        summary = run_mu_sweep(
            case,
            args.dynamic_simulation,
            args.output_root,
            mu_values=mu_values,
            tmax=args.tmax,
            rel_tol=args.rel_tol,
            abs_tol=args.abs_tol,
            ida_max_steps=args.ida_max_steps,
            keep_csv=args.keep_csv,
            fault_window=fault_window,
        )
        print(f"  wrote {summary}")
    return 0


def command_fault_sweep(args: argparse.Namespace) -> int:
    status = 0
    for case in by_keys(args.cases):
        study_path = generate_fault_sweep(case, args.output_root, tmax=args.tmax, ida_max_steps=args.ida_max_steps)
        if args.run:
            result = run_binary(args.contingency_analysis, study_path)
            status = max(status, result.returncode)
    return status


def command_plot(args: argparse.Namespace) -> int:
    cases = by_keys(args.cases)
    figures = args.figures_dir or (args.output_root / "figures")
    if args.kind == "step-size":
        plot_step_size(cases, args.output_root, args.output or figures / "step_size.png", args.show)
    elif args.kind == "contingency":
        plot_contingency(cases, args.output_root, figures, include_failed=args.include_failed, show=args.show)
    elif args.kind == "ctg-frontier":
        plot_contingency_frontier(
            cases,
            args.output_root,
            args.output or figures / "ctg_effort_frontier.png",
            include_failed=args.include_failed,
            show=args.show,
        )
    elif args.kind == "cumulative-work":
        plot_cumulative_work(cases, args.output_root, args.output or figures / "cumulative_work_all_cases.png", args.show)
    elif args.kind == "work-per-step":
        plot_work_per_step(cases, args.output_root, args.output or figures / "work_per_step_vs_stepsize.png", args.show)
    elif args.kind == "tolerance":
        plot_tolerance(cases, args.output_root, args.output or figures / "tolerance_work.png", args.show)
    elif args.kind == "tol-step-size":
        plot_tolerance_step_size(cases, args.output_root, figures, output=args.output, show=args.show, cmap=args.cmap)
    elif args.kind == "mu":
        plot_mu(cases, args.output_root, figures, args.show)
    elif args.kind == "signals":
        plot_signals(cases, args.output_root, figures, args.quantity, args.show)
    else:
        raise SystemExit(f"Unhandled plot kind: {args.kind}")
    return 0


def command_table(args: argparse.Namespace) -> int:
    case_keys = args.cases if args.cases else ["newengland", "hawaii", "illinois", "wecc", "texas"]
    cases = by_keys(case_keys)
    figures = args.figures_dir or (args.output_root / "figures")
    if args.kind == "errors":
        write_error_table(
            cases,
            args.output_root,
            args.output or figures / "error_table.tex",
            use_publication_labels=not bool(args.cases),
        )
    else:
        raise SystemExit(f"Unhandled table kind: {args.kind}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="Run DynamicSimulation and/or ContingencyAnalysis.")
    add_common_case_args(run)
    run.add_argument("--steps", action="store_true", help="Run DynamicSimulation with IDA step history.")
    run.add_argument("--contingency", action="store_true", help="Run ContingencyAnalysis.")
    run.add_argument("--all-sweeps", action="store_true", help="Run large-case contingency sweeps by default.")
    run.add_argument("--output-csv", action="store_true", help="Write monitor CSV and error JSON when a reference exists.")
    run.add_argument("--dynamic-simulation", type=Path, default=DEFAULT_DYNAMIC_SIMULATION)
    run.add_argument("--contingency-analysis", type=Path, default=DEFAULT_CONTINGENCY_ANALYSIS)
    run.set_defaults(func=command_run)

    tol = subparsers.add_parser("tol-sweep", help="Run DynamicSimulation across IDA tolerances.")
    add_common_case_args(tol)
    tol.add_argument("--rtols", nargs="+", type=float, default=[1.0e-3, 1.0e-5, 1.0e-7, 1.0e-9])
    tol.add_argument("--all-decades", action="store_true", help="Use every decade 1e-3 through 1e-11.")
    tol.add_argument("--atol-ratio", type=float, default=1.0e-2)
    tol.add_argument("--repeats", type=int, default=1, help="Runs per tolerance; minimum wall time is kept.")
    tol.add_argument("--no-output-csv", action="store_true", help="Do not write monitor/error outputs.")
    tol.add_argument("--dynamic-simulation", type=Path, default=DEFAULT_DYNAMIC_SIMULATION)
    tol.set_defaults(func=command_tol_sweep)

    mu = subparsers.add_parser("mu-sweep", help="Run reference-error sweep over CommonMath mu.")
    add_common_case_args(mu)
    mu.add_argument("--mu-values", nargs="+", type=float, default=None)
    mu.add_argument("--mu-min", type=float, default=10.0)
    mu.add_argument("--mu-max", type=float, default=1.0e4)
    mu.add_argument("--mu-count", type=int, default=17)
    mu.add_argument("--keep-csv", action="store_true", help="Keep per-mu monitor CSVs.")
    mu.add_argument("--dynamic-simulation", type=Path, default=DEFAULT_DYNAMIC_SIMULATION)
    mu.set_defaults(func=command_mu_sweep)

    fault = subparsers.add_parser("fault-sweep", help="Generate one-BusFault-per-bus copied case studies.")
    fault.add_argument("--cases", nargs="*", default=["wecc"], help="Case keys. Default: wecc.")
    fault.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    fault.add_argument("--tmax", type=float, default=None)
    fault.add_argument("--ida-max-steps", type=int, default=None,
                       help="Override IDA maximum internal steps per solve call.")
    fault.add_argument("--run", action="store_true", help="Run ContingencyAnalysis after writing the sweep.")
    fault.add_argument("--contingency-analysis", type=Path, default=DEFAULT_CONTINGENCY_ANALYSIS)
    fault.set_defaults(func=command_fault_sweep)

    plot = subparsers.add_parser("plot", help="Plot generated validation outputs.")
    plot.add_argument(
        "kind",
        choices=(
            "step-size",
            "contingency",
            "ctg-frontier",
            "cumulative-work",
            "work-per-step",
            "tolerance",
            "tol-step-size",
            "mu",
            "signals",
        ),
    )
    plot.add_argument("--cases", nargs="*", default=[], help="Case keys. Default: all validation cases.")
    plot.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    plot.add_argument("--figures-dir", type=Path, default=None)
    plot.add_argument("--output", type=Path, default=None, help="Single-output path where supported.")
    plot.add_argument("--include-failed", action="store_true", help="Include failed contingencies in histograms.")
    plot.add_argument("--cmap", default=None, help="Sequential colormap for tol-step-size plots.")
    plot.add_argument("--quantity", choices=("voltage", "speed"), default="voltage", help="Signal quantity for plot signals.")
    plot.add_argument("--show", action="store_true")
    plot.set_defaults(func=command_plot)

    table = subparsers.add_parser("table", help="Write generated validation tables.")
    table.add_argument("kind", choices=("errors",))
    table.add_argument("--cases", nargs="*", default=[], help="Case keys. Default: publication table order.")
    table.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    table.add_argument("--figures-dir", type=Path, default=None)
    table.add_argument("--output", type=Path, default=None)
    table.set_defaults(func=command_table)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    args.output_root = args.output_root.expanduser()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
