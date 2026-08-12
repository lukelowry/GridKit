#!/usr/bin/env python3
"""Aggregate the smooth-vs-piecewise study matrix into results.csv.

Per run: solver work counters (ida_stats.json), step/LTE statistics
(ida_steps.json), wall time and profile buckets (stdout.txt), and trajectory
errors against two references on the shared monitor grid:
  ref_model : the case's tight-tol piecewise arm (model + integration error)
  ref_self  : tight-tol same-mode same-mu          (pure integration error)

Trajectory error is reported per run as the WRMS and max of the pointwise
relative difference over all shared numeric columns of study_out.csv, with a
per-column absolute floor to keep near-zero signals from dominating.
"""

import csv
import json
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORK = HERE / "work"
ABS_FLOOR = 1.0e-6


def read_profile_block(text: str, begin: str, end: str) -> dict:
    m = re.search(begin + r"\n(.*?)" + end, text, re.S)
    if not m:
        return {}
    out = {}
    for line in m.group(1).strip().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            try:
                out[k] = float(v)
            except ValueError:
                pass
    return out


def read_mon(path: Path):
    if not path.exists():
        return None, None
    with open(path) as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return None, None
        rows = [[float(x) for x in r] for r in reader if r]
    return header, rows


def trajectory_error(run_dir: Path, ref_dir: Path):
    """WRMS/max relative error between the two runs' mon.csv on shared columns."""
    h1, r1 = read_mon(run_dir / "study_out.csv")
    h2, r2 = read_mon(ref_dir / "study_out.csv")
    if h1 is None or h2 is None or h1 != h2 or not r1 or len(r1) != len(r2):
        return None, None
    total = 0.0
    count = 0
    worst = 0.0
    for a, b in zip(r1, r2):
        for j in range(1, len(a)):  # column 0 is time
            scale = max(abs(b[j]), ABS_FLOOR)
            e = abs(a[j] - b[j]) / scale
            total += e * e
            count += 1
            worst = max(worst, e)
    if count == 0:
        return None, None
    return math.sqrt(total / count), worst


def load_stats(run_dir: Path) -> dict:
    out = {}
    p = run_dir / "ida_stats.json"
    if p.exists():
        with open(p) as f:
            s = json.load(f)
        out["steps"] = s["integrator"]["steps"]
        out["residual_evals"] = s["integrator"]["residual_evals"]
        out["linear_solver_setups"] = s["integrator"]["linear_solver_setups"]
        out["error_test_failures"] = s["integrator"]["error_test_failures"]
        out["nonlinear_iterations"] = s["nonlinear_solver"]["iterations"]
        out["nonlinear_convergence_failures"] = s["nonlinear_solver"]["convergence_failures"]
        out["jacobian_evals"] = s["linear_solver"]["jacobian_evals"]
        out["segment_count"] = s["segment_count"]
    p = run_dir / "ida_steps.json"
    if p.exists():
        with open(p) as f:
            s = json.load(f)
        h = []
        lte = []
        orders = []
        for seg in s.get("segments", []):
            for step in seg.get("steps", []):
                h.append(step["last_step"])
                lte.append(step.get("lte_wrms", 0.0))
                orders.append(step["last_order"])
        if h:
            out["accepted_steps"] = len(h)
            out["h_min"] = min(h)
            out["h_max"] = max(h)
            out["h_geomean"] = math.exp(sum(math.log(x) for x in h if x > 0) / max(1, sum(1 for x in h if x > 0)))
            out["lte_wrms_mean"] = sum(lte) / len(lte)
            out["lte_wrms_max"] = max(lte)
            out["order_mean"] = sum(orders) / len(orders)
    return out


def main() -> None:
    with open(WORK / "manifest.json") as f:
        manifest = json.load(f)

    by_key = {(e["case"], e["mode"], e["mu"], e["tight"]): e for e in manifest}
    rows = []
    for e in manifest:
        run_dir = WORK / e["dir"]
        result_file = run_dir / "result.json"
        if not result_file.exists():
            continue
        with open(result_file) as f:
            result = json.load(f)

        row = {
            "case": e["case"], "tier": e["tier"], "mode": e["mode"],
            "mu": e["mu"], "tight": e["tight"],
            "returncode": result.get("returncode"),
            "wall_seconds": result.get("wall_seconds"),
        }
        stdout = (run_dir / "stdout.txt").read_text() if (run_dir / "stdout.txt").exists() else ""
        row.update({f"profile_{k}": v for k, v in read_profile_block(
            stdout, "GRIDKIT_PROFILE_BEGIN", "GRIDKIT_PROFILE_END").items()})
        sysm = re.search(r"states=(\d+)", stdout)
        if sysm:
            row["states"] = int(sysm.group(1))
        row.update(load_stats(run_dir))

        if not e["tight"] and result.get("returncode") == 0:
            # One piecewise reference arm per case
            ref_model = next((x for x in manifest
                              if x["case"] == e["case"] and x["mode"] == "piecewise"
                              and x["tight"]), None)
            ref_self = by_key.get((e["case"], e["mode"], e["mu"], True))
            for tag, ref in (("ref_model", ref_model), ("ref_self", ref_self)):
                if ref is None:
                    continue
                ref_dir = WORK / ref["dir"]
                if (ref_dir / "result.json").exists():
                    wrms, worst = trajectory_error(run_dir, ref_dir)
                    row[f"{tag}_wrms"] = wrms
                    row[f"{tag}_max"] = worst
        rows.append(row)

    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    out = HERE / "results.csv"
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} runs -> {out}")


if __name__ == "__main__":
    main()
