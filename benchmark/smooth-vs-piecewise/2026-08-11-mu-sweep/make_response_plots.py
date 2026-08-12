#!/usr/bin/env python3
"""Render the time-series response plots with the standing pdsim tooling.

Uses scripts/plot_dynamic_simulation_responses.py (side-by-side + difference
panels for two runs: bus voltages, rotor angles, frequency deviations) for
every case where the piecewise arm completes, comparing it against smooth
mu = 10^2 and mu = 10^4, and scripts/pdsim/plot_bus_frequency.py (per-bus
frequency from unwrapped Va, one figure per run) for ACTIVSg10k, whose
piecewise arm cannot be paired (it aborts, so the time axes differ).

Outputs under figures/responses/<case>/.
"""

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
WORK = HERE / "work"
OUT = HERE / "figures" / "responses"

COMPARE = REPO / "scripts/plot_dynamic_simulation_responses.py"
BUSFREQ = REPO / "scripts/pdsim/plot_bus_frequency.py"

CASES = ["TwoBusTgov1", "TwoBusGensal", "TwoBusIeeet1", "ThreeBusBasic",
         "ThreeBusGenrouSat", "TwoBusGensalSat", "TwoBusTgov1Bnd",
         "TwoBusIeeet1Bnd", "TwoBusGensalBnd", "ThreeBusGenrouBnd",
         "ACTIVSg200", "ACTIVSg500", "WECC240"]
SMOOTH_ARMS = [("mu100", "smooth-mu100", r"smooth $\mu = 10^2$"),
               ("mu10000", "smooth-mu10000", r"smooth $\mu = 10^4$")]
TENK_FREQ_ARMS = ["piecewise-mu10000", "smooth-mu10",
                  "smooth-mu100", "smooth-mu1000", "smooth-mu10000"]


def run(cmd, **kw):
    r = subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)
    if r.returncode != 0:
        print(r.stdout[-800:], file=sys.stderr)
        print(r.stderr[-800:], file=sys.stderr)
        raise SystemExit(f"failed: {' '.join(str(c) for c in cmd[:4])} ...")
    return r


def solver_json(case, arm):
    return next((WORK / case / arm).glob("*.solver.json"))


def main():
    for case in CASES:
        pw_csv = WORK / case / "piecewise-mu10000" / "study_out.csv"
        for tag, arm, label in SMOOTH_ARMS:
            out = OUT / case / f"piecewise-vs-{tag}"
            run(["python3", COMPARE,
                 "--case", case,
                 "--study", solver_json(case, "piecewise-mu10000"),
                 "--develop-csv", pw_csv,
                 "--branch-csv", WORK / case / arm / "study_out.csv",
                 "--output-dir", out,
                 "--develop-label", "piecewise (exact)",
                 "--branch-label", label,
                 "--omega-unit", "pu"])
            print(f"{case}: piecewise vs {tag} -> {out.relative_to(HERE)}")

    # ACTIVSg10k: the smooth pair compares; the piecewise arm aborts, so its
    # frequencies are per-run figures.
    case = "ACTIVSg10k"
    out = OUT / case / "mu100-vs-mu10000"
    run(["python3", COMPARE,
         "--case", case,
         "--study", solver_json(case, "smooth-mu100"),
         "--develop-csv", WORK / case / "smooth-mu100" / "study_out.csv",
         "--branch-csv", WORK / case / "smooth-mu10000" / "study_out.csv",
         "--output-dir", out,
         "--develop-label", r"smooth $\mu = 10^2$",
         "--branch-label", r"smooth $\mu = 10^4$",
         "--omega-unit", "pu"])
    print(f"{case}: mu100 vs mu10000 -> {out.relative_to(HERE)}")

    for arm in TENK_FREQ_ARMS:
        out = OUT / case
        out.mkdir(parents=True, exist_ok=True)
        dest = out / f"bus_frequency_{arm.replace('-', '_')}.png"
        run(["python3", BUSFREQ, solver_json(case, arm),
             "--input", WORK / case / arm / "study_out.csv",
             "--output", dest,
             "--ylim", "59.94", "60.06"],
            cwd=REPO / "scripts/pdsim")
        print(f"{case}: {arm} bus frequency -> {dest.relative_to(HERE)}")


if __name__ == "__main__":
    main()
