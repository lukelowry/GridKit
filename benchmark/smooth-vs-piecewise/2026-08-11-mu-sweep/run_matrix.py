#!/usr/bin/env python3
"""Execute the smooth-vs-piecewise study matrix.

Runs DynamicSimulation for every variant in work/manifest.json, pinned to one
core, capturing stdout (GRIDKIT_PROFILE / SYSTEM_RESIDUAL_PROFILE blocks),
wall time, exit status, and the ida_stats/ida_steps JSON written by the app.
Each run directory receives stdout.txt, stderr.txt, and result.json.

Already-completed runs (result.json present with matching config) are skipped
unless --force is given, so the matrix can be resumed.
"""

import argparse
import json
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
DEFAULT_APP = REPO / "build/application/PhasorDynamics/DynamicSimulation"


def run_variant(app: Path, entry: dict, work: Path, cpu: str, timeout: float, force: bool) -> dict:
    run_dir = work / entry["dir"]
    solver = next(run_dir.glob("*.solver.json"))
    result_file = run_dir / "result.json"

    if result_file.exists() and not force:
        with open(result_file) as f:
            prior = json.load(f)
        if prior.get("returncode") == 0:
            return prior

    cmd = ["taskset", "-c", cpu, str(app), solver.name]
    start = time.monotonic()
    try:
        proc = subprocess.run(cmd, cwd=run_dir, capture_output=True, text=True, timeout=timeout)
        returncode = proc.returncode
        stdout, stderr = proc.stdout, proc.stderr
        timed_out = False
    except subprocess.TimeoutExpired as ex:
        returncode = -1
        stdout = ex.stdout or ""
        stderr = (ex.stderr or "") + "\nTIMEOUT"
        timed_out = True
    wall = time.monotonic() - start

    (run_dir / "stdout.txt").write_text(stdout)
    (run_dir / "stderr.txt").write_text(stderr)

    result = dict(entry)
    result.update({
        "returncode": returncode,
        "timed_out": timed_out,
        "wall_seconds": wall,
    })
    with open(result_file, "w") as f:
        json.dump(result, f, indent=2)
        f.write("\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work", default=str(HERE / "work"))
    parser.add_argument("--app", default=str(DEFAULT_APP))
    parser.add_argument("--cpu", default="4", help="core to pin runs to")
    parser.add_argument("--timeout", type=float, default=3600.0, help="per-run timeout seconds")
    parser.add_argument("--cases", nargs="*", default=None, help="subset of cases")
    parser.add_argument("--tiers", nargs="*", default=None, help="subset of tiers (dense/coarse)")
    parser.add_argument("--force", action="store_true", help="rerun completed variants")
    args = parser.parse_args()

    work = Path(args.work)
    with open(work / "manifest.json") as f:
        manifest = json.load(f)

    selected = [e for e in manifest
                if (args.cases is None or e["case"] in args.cases)
                and (args.tiers is None or e["tier"] in args.tiers)]

    app = Path(args.app)
    failures = 0
    for i, entry in enumerate(selected, 1):
        r = run_variant(app, entry, work, args.cpu, args.timeout, args.force)
        status = "ok" if r["returncode"] == 0 else f"rc={r['returncode']}"
        print(f"[{i}/{len(selected)}] {entry['dir']}: {status} ({r['wall_seconds']:.1f}s)", flush=True)
        if r["returncode"] != 0:
            failures += 1

    print(f"done: {len(selected) - failures}/{len(selected)} ok")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
