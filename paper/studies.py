"""Generate temporary studies and run PhasorDynamics validation workflows."""

from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path
from typing import Any

from cases import BUILD_DIR, REPO_ROOT, Case, load_json, load_study, reference_path, resolve_model_path
from signals import write_error_report


DEFAULT_OUTPUT_ROOT = BUILD_DIR / "validation"
DEFAULT_DYNAMIC_SIMULATION = BUILD_DIR / "application" / "PhasorDynamics" / "DynamicSimulation"
DEFAULT_CONTINGENCY_ANALYSIS = BUILD_DIR / "application" / "PhasorDynamics" / "ContingencyAnalysis"
DEFAULT_CONTINGENCY_MAX_STEPS = 50000


def validate_tolerance_pair(rel_tol: float | None, abs_tol: float | None) -> None:
    if rel_tol is None and abs_tol is None:
        return
    if rel_tol is None or abs_tol is None:
        raise SystemExit("--rel-tol and --abs-tol must be provided together")
    if not (rel_tol > 0.0 and math.isfinite(rel_tol)):
        raise SystemExit("--rel-tol must be positive and finite")
    if not (abs_tol > 0.0 and math.isfinite(abs_tol)):
        raise SystemExit("--abs-tol must be positive and finite")


def apply_tolerances(study: dict[str, Any], rel_tol: float | None, abs_tol: float | None) -> None:
    validate_tolerance_pair(rel_tol, abs_tol)
    if rel_tol is not None and abs_tol is not None:
        study["rel_tol"] = rel_tol
        study["abs_tol"] = abs_tol


def apply_ida_max_steps(study: dict[str, Any], ida_max_steps: int | None) -> None:
    if ida_max_steps is None:
        return
    if ida_max_steps <= 0:
        raise SystemExit("--ida-max-steps must be positive")
    study["ida_max_steps"] = ida_max_steps


def normalize_events(events: list[dict[str, Any]], fault_window: tuple[float, float] | None) -> list[dict[str, Any]]:
    copied = [dict(event) for event in events]
    if fault_window is None:
        return copied
    fault_on, fault_off = fault_window
    for event in copied:
        if event.get("type") == "fault_on":
            event["time"] = fault_on
        elif event.get("type") == "fault_off":
            event["time"] = fault_off
    return copied


def base_study(
    case: Case,
    *,
    tmax: float | None = None,
    mu: float | None = None,
    rel_tol: float | None = None,
    abs_tol: float | None = None,
    ida_max_steps: int | None = None,
    fault_window: tuple[float, float] | None = None,
) -> dict[str, Any]:
    source = load_study(case)
    study: dict[str, Any] = {
        "system_model_file": str(resolve_model_path(case, source).resolve()),
        "dt": source["dt"],
        "tmax": source["tmax"] if tmax is None else tmax,
        "events": normalize_events(list(source.get("events", [])), fault_window),
    }
    for key in ("ida_max_order", "ida_max_steps", "ida_max_dt", "mu"):
        if key in source:
            study[key] = source[key]
    if mu is not None:
        if not (mu > 0.0 and math.isfinite(mu)):
            raise SystemExit("--mu must be positive and finite")
        study["mu"] = mu
    apply_tolerances(study, rel_tol, abs_tol)
    apply_ida_max_steps(study, ida_max_steps)
    return study


def write_study(study: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(study, indent=2) + "\n", encoding="utf-8")
    return path


def ensure_binary(path: Path) -> Path:
    path = path.expanduser()
    if not path.exists():
        raise SystemExit(f"Binary not found: {path}\nBuild it first with cmake --build build")
    return path


def run_binary(binary: Path, study_path: Path, *, capture: bool = False) -> subprocess.CompletedProcess:
    binary = ensure_binary(binary)
    print(f"  $ {binary.name} {study_path}")
    return subprocess.run(
        [str(binary), str(study_path)],
        cwd=study_path.parent,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=False,
    )


def run_dynamic_case(
    case: Case,
    dynamic_simulation: Path,
    output_root: Path,
    *,
    record_steps: bool = True,
    record_stats: bool = True,
    output_csv: bool = False,
    tmax: float | None = None,
    mu: float | None = None,
    rel_tol: float | None = None,
    abs_tol: float | None = None,
    ida_max_steps: int | None = None,
    fault_window: tuple[float, float] | None = None,
) -> subprocess.CompletedProcess:
    case_output = output_root / case.key
    study = base_study(
        case,
        tmax=tmax,
        mu=mu,
        rel_tol=rel_tol,
        abs_tol=abs_tol,
        ida_max_steps=ida_max_steps,
        fault_window=fault_window,
    )
    if record_stats:
        study["ida_stats"] = str(case_output / f"{case.key}.ida_stats.json")
    if record_steps:
        study["ida_steps"] = str(case_output / f"{case.key}.ida_steps.json")
    ref = reference_path(case)
    output_path = case_output / f"{case.key}.csv"
    if output_csv and ref is not None:
        study["output_file"] = str(output_path)
    study_path = write_study(study, case_output / f"{case.key}.solver.json")
    result = run_binary(dynamic_simulation, study_path)
    if output_csv and ref is not None and output_path.exists():
        write_error_report(case.key, output_path, ref, case_output / f"{case.key}.error.json")
    return result


def run_contingency_case(
    case: Case,
    contingency_analysis: Path,
    output_root: Path,
    *,
    tmax: float | None = None,
    mu: float | None = None,
    rel_tol: float | None = None,
    abs_tol: float | None = None,
    ida_max_steps: int | None = None,
    fault_window: tuple[float, float] | None = None,
) -> subprocess.CompletedProcess:
    case_output = output_root / case.key
    step_budget = DEFAULT_CONTINGENCY_MAX_STEPS if ida_max_steps is None else ida_max_steps
    study = base_study(
        case,
        tmax=tmax,
        mu=mu,
        rel_tol=rel_tol,
        abs_tol=abs_tol,
        ida_max_steps=step_budget,
        fault_window=fault_window,
    )
    study["ida_stats"] = str(case_output / f"{case.key}.ida_stats.json")
    study_path = write_study(study, case_output / f"{case.key}.ctg.solver.json")
    return run_binary(contingency_analysis, study_path)


def rtol_tag(rtol: float) -> str:
    return f"rtol_{rtol:.0e}"


def mu_tag(mu: float) -> str:
    return "mu_" + f"{mu:.12g}".replace("+", "").replace("-", "m").replace(".", "p")


def run_tolerance_sweep(
    case: Case,
    dynamic_simulation: Path,
    output_root: Path,
    *,
    rtols: list[float],
    atol_ratio: float,
    repeats: int,
    tmax: float | None = None,
    mu: float | None = None,
    ida_max_steps: int | None = None,
    output_csv: bool = True,
    fault_window: tuple[float, float] | None = None,
) -> None:
    sweep_dir = output_root / case.key / "tol_sweep"
    ref = reference_path(case)
    for rtol in rtols:
        atol = rtol * atol_ratio
        tag = rtol_tag(rtol)
        study = base_study(
            case,
            tmax=tmax,
            mu=mu,
            rel_tol=rtol,
            abs_tol=atol,
            ida_max_steps=ida_max_steps,
            fault_window=fault_window,
        )
        stats_file = sweep_dir / f"{tag}.ida_stats.json"
        study["ida_stats"] = str(stats_file)
        study["ida_steps"] = str(sweep_dir / f"{tag}.ida_steps.json")
        output_path = sweep_dir / f"{tag}.csv"
        if output_csv and ref is not None:
            study["output_file"] = str(output_path)
        study_path = write_study(study, sweep_dir / f"{tag}.solver.json")
        print(f"[{case.key}] rel_tol={rtol:g} abs_tol={atol:g}")
        wall_times: list[float] = []
        for _ in range(max(1, repeats)):
            run_binary(dynamic_simulation, study_path)
            if stats_file.exists():
                stats = load_json(stats_file)
                wall = stats.get("timing", {}).get("wall_clock_seconds")
                if wall is not None:
                    wall_times.append(float(wall))
        if repeats > 1 and wall_times and stats_file.exists():
            stats = load_json(stats_file)
            stats.setdefault("timing", {})["wall_clock_seconds"] = min(wall_times)
            stats_file.write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
        if output_csv and ref is not None and output_path.exists():
            write_error_report(case.key, output_path, ref, sweep_dir / f"{tag}.error.json")


def log_spaced(minimum: float, maximum: float, count: int) -> list[float]:
    if not (minimum > 0.0 and maximum > minimum and count >= 2):
        raise SystemExit("mu range must satisfy minimum > 0, maximum > minimum, count >= 2")
    ratio = maximum / minimum
    return [float(f"{minimum * ratio ** (index / (count - 1)):.8g}") for index in range(count)]


def run_mu_sweep(
    case: Case,
    dynamic_simulation: Path,
    output_root: Path,
    *,
    mu_values: list[float],
    tmax: float | None = None,
    rel_tol: float | None = None,
    abs_tol: float | None = None,
    ida_max_steps: int | None = None,
    keep_csv: bool = False,
    fault_window: tuple[float, float] | None = None,
) -> Path:
    validate_tolerance_pair(rel_tol, abs_tol)
    tolerance_tag = "model_default" if rel_tol is None else f"{rtol_tag(rel_tol)}_atol_{abs_tol:.0e}"
    sweep_dir = output_root / case.key / "mu_sweep" / tolerance_tag
    ref = reference_path(case)
    if ref is None:
        raise SystemExit(f"{case.key} has no reference_file; mu sweep needs a reference CSV")
    points = []
    for mu in mu_values:
        tag = mu_tag(mu)
        output_path = sweep_dir / f"{tag}.csv"
        study = base_study(
            case,
            tmax=tmax,
            mu=mu,
            rel_tol=rel_tol,
            abs_tol=abs_tol,
            ida_max_steps=ida_max_steps,
            fault_window=fault_window,
        )
        study["output_file"] = str(output_path)
        study_path = write_study(study, sweep_dir / f"{tag}.solver.json")
        print(f"[{case.key}] mu={mu:g}")
        result = run_binary(dynamic_simulation, study_path)
        if result.returncode != 0:
            raise SystemExit(f"DynamicSimulation failed for {study_path} with exit code {result.returncode}")
        error_path = sweep_dir / f"{tag}.error.json"
        report = write_error_report(case.key, output_path, ref, error_path)
        if not keep_csv and output_path.exists():
            output_path.unlink()
        points.append({
            "mu": mu,
            "tag": tag,
            "solver_json": str(study_path),
            "error_json": str(error_path),
            "error": report,
        })
    summary = {
        "schema": "gridkit.validation.mu_sweep.v1",
        "case": case.key,
        "source_solver_json": str(case.solver_json),
        "reference": str(ref),
        "tolerance": {"rel_tol": rel_tol, "abs_tol": abs_tol},
        "points": points,
    }
    summary_path = sweep_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary_path


def generate_fault_sweep(
    case: Case,
    output_root: Path,
    *,
    tmax: float | None = None,
    ida_max_steps: int | None = None,
) -> Path:
    source = load_study(case)
    model_path = resolve_model_path(case, source)
    model = load_json(model_path)
    devices = list(model.get("devices", []))
    template = next((device for device in devices if device.get("class") == "BusFault"), None)
    params = dict((template or {}).get("params", {"state0": False, "R": 0.0, "X": 0.001}))
    kept = [device for device in devices if device.get("class") != "BusFault"]
    faults = [
        {
            "class": "BusFault",
            "ports": {"bus": bus["number"]},
            "id": f"fault_{index + 1}",
            "params": params,
        }
        for index, bus in enumerate(model.get("buses", []))
    ]
    swept = {key: value for key, value in model.items() if key != "devices"}
    swept["devices"] = kept + faults

    case_output = output_root / case.key
    case_output.mkdir(parents=True, exist_ok=True)
    swept_model_path = case_output / f"{case.key}.sweep.case.json"
    swept_model_path.write_text(json.dumps(swept, indent=2) + "\n", encoding="utf-8")

    study = {
        "system_model_file": str(swept_model_path.resolve()),
        "dt": source["dt"],
        "tmax": source["tmax"] if tmax is None else tmax,
        "events": list(source.get("events", [])),
        "ida_stats": str(case_output / f"{case.key}.ida_stats.json"),
    }
    for key in ("ida_max_order", "ida_max_steps", "ida_max_dt", "mu"):
        if key in source:
            study[key] = source[key]
    apply_ida_max_steps(study, DEFAULT_CONTINGENCY_MAX_STEPS if ida_max_steps is None else ida_max_steps)
    study_path = write_study(study, case_output / f"{case.key}.sweep.solver.json")
    print(f"[{case.key}] wrote {len(faults)}-fault swept study: {study_path}")
    return study_path
