"""Loaders for IDA diagnostics and validation sweep output."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def case_dir(output_root: Path, case_key: str) -> Path:
    return output_root / case_key


def step_history_path(output_root: Path, case_key: str) -> Path:
    return case_dir(output_root, case_key) / f"{case_key}.ida_steps.json"


def stats_path(output_root: Path, case_key: str) -> Path:
    return case_dir(output_root, case_key) / f"{case_key}.ida_stats.json"


def contingency_summary_path(output_root: Path, case_key: str) -> Path:
    return case_dir(output_root, case_key) / "results" / "ContingencyAnalysis" / "summary.json"


def tol_sweep_dir(output_root: Path, case_key: str) -> Path:
    return case_dir(output_root, case_key) / "tol_sweep"


def mu_sweep_dir(output_root: Path, case_key: str) -> Path:
    return case_dir(output_root, case_key) / "mu_sweep"


def finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def counter(data: dict[str, Any], group: str, key: str) -> int:
    try:
        return int(data.get(group, {}).get(key, 0))
    except (TypeError, ValueError):
        return 0


@dataclass(frozen=True)
class StepSample:
    time: float
    step_size: float
    accepted_steps: int
    residual_evals: int
    jacobian_evals: int
    nonlinear_iterations: int
    error_test_failures: int


@dataclass(frozen=True)
class StepHistory:
    case_key: str
    path: Path
    samples: list[StepSample]
    boundaries: list[float]


@dataclass(frozen=True)
class ContingencyEffort:
    fault_id: int
    passed: bool
    solve_status_name: str
    steps: int
    residual_evals: int
    jacobian_evals: int
    nonlinear_iterations: int
    error_test_failures: int


@dataclass(frozen=True)
class TolerancePoint:
    rel_tol: float
    abs_tol: float
    steps: int
    residual_evals: int
    jacobian_evals: int
    wall_clock_seconds: float | None
    omega_rmse: float | None
    vm_rmse: float | None


def load_step_history(output_root: Path, case_key: str) -> StepHistory | None:
    path = step_history_path(output_root, case_key)
    if not path.exists():
        return None
    data = load_json(path)
    samples: list[StepSample] = []
    boundaries: set[float] = set()
    for segment in data.get("segments", []):
        for key in ("start_time", "end_time"):
            value = finite_float(segment.get(key))
            if value is not None:
                boundaries.add(round(value, 12))
        for raw in segment.get("steps", []):
            time = finite_float(raw.get("step_end_time"))
            step_size = finite_float(raw.get("last_step"))
            counters = raw.get("counter_delta", {})
            if time is None or step_size is None:
                continue
            step_size = abs(step_size)
            accepted_steps = counter(counters, "integrator", "steps")
            if step_size <= 0.0 or accepted_steps <= 0:
                continue
            samples.append(
                StepSample(
                    time=time,
                    step_size=step_size,
                    accepted_steps=accepted_steps,
                    residual_evals=counter(counters, "integrator", "residual_evals"),
                    jacobian_evals=counter(counters, "linear_solver", "jacobian_evals"),
                    nonlinear_iterations=counter(counters, "nonlinear_solver", "iterations"),
                    error_test_failures=counter(counters, "integrator", "error_test_failures"),
                )
            )
    return StepHistory(case_key, path, samples, sorted(boundaries))


def load_contingency_effort(output_root: Path, case_key: str, include_failed: bool = False) -> list[ContingencyEffort]:
    summary_path = contingency_summary_path(output_root, case_key)
    if not summary_path.exists():
        return []
    summary = load_json(summary_path)
    base = summary_path.parent
    rows: list[ContingencyEffort] = []
    for fault in summary.get("faults", []):
        passed = bool(fault.get("passed"))
        if not passed and not include_failed:
            continue
        stats_ref = fault.get("ida_stats")
        if not stats_ref:
            continue
        path = base / stats_ref
        if not path.exists():
            continue
        stats = load_json(path)
        steps = counter(stats, "integrator", "steps")
        if steps <= 0:
            continue
        rows.append(
            ContingencyEffort(
                fault_id=int(fault.get("fault_id", 0)),
                passed=passed,
                solve_status_name=str(fault.get("solve_status_name", "")),
                steps=steps,
                residual_evals=counter(stats, "integrator", "residual_evals"),
                jacobian_evals=counter(stats, "linear_solver", "jacobian_evals"),
                nonlinear_iterations=counter(stats, "nonlinear_solver", "iterations"),
                error_test_failures=counter(stats, "integrator", "error_test_failures"),
            )
        )
    return rows


def load_tolerance_points(output_root: Path, case_key: str) -> list[TolerancePoint]:
    root = tol_sweep_dir(output_root, case_key)
    points: list[TolerancePoint] = []
    for path in sorted(root.glob("rtol_*.ida_stats.json")):
        stats = load_json(path)
        config = stats.get("config", {})
        rel_tol = finite_float(config.get("rel_tol"))
        abs_tol = finite_float(config.get("abs_tol"))
        if rel_tol is None or abs_tol is None:
            continue
        error_path = path.with_name(path.name.replace(".ida_stats.json", ".error.json"))
        error = load_json(error_path) if error_path.exists() else {}
        points.append(
            TolerancePoint(
                rel_tol=rel_tol,
                abs_tol=abs_tol,
                steps=counter(stats, "integrator", "steps"),
                residual_evals=counter(stats, "integrator", "residual_evals"),
                jacobian_evals=counter(stats, "linear_solver", "jacobian_evals"),
                wall_clock_seconds=finite_float(stats.get("timing", {}).get("wall_clock_seconds")),
                omega_rmse=finite_float((error.get("omega") or {}).get("rmse")),
                vm_rmse=finite_float((error.get("vm") or {}).get("rmse")),
            )
        )
    points.sort(key=lambda item: item.rel_tol, reverse=True)
    return points
