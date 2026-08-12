#!/usr/bin/env python3
"""Generate the smooth-vs-piecewise study matrix.

Each variant is a self-contained run directory with a generated solver.json:
  work/<case>/<mode>-mu<tag>[-tight]/<case>.solver.json

Grid per tier:
  dense : smooth x 9 half-decade mu in [1e0, 1e4], default and tight tolerance
  coarse: smooth x {1e0, 1e2, 1e4},                default and tight tolerance
  both  : one pinned piecewise reference arm per case (gates at GATE_MU_PIN),
          default and tight tolerance

The tight-tolerance piecewise run is the exact-model accuracy reference for
every smooth run of its case; tight-tolerance runs of the same (mode, mu)
isolate pure integration error from smoothing/model error.
"""

import argparse
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]

MU_DENSE = [1.0, 3.1622776601683795, 10.0, 31.622776601683793, 100.0,
            316.22776601683796, 1000.0, 3162.2776601683795, 10000.0]
MU_COARSE = [1.0, 100.0, 10000.0]

TIGHT_REL_TOL = 1.0e-9
TIGHT_ABS_TOL = 1.0e-11

# The piecewise arm is one fixed reference model per case: its ramp family is
# exact at any mu and its PWL step gates are pinned to this slope.
GATE_MU_PIN = 10000.0

# case name -> (source solver.json, tier)
CASES = {
    "TwoBusTgov1": ("examples/PhasorDynamics/Tiny/TwoBus/Tgov1/TwoBusTgov1.solver.json", "dense"),
    "TwoBusGensal": ("examples/PhasorDynamics/Tiny/TwoBus/Gensal/TwoBusGensal.solver.json", "dense"),
    "TwoBusIeeet1": ("examples/PhasorDynamics/Tiny/TwoBus/Ieeet1/TwoBusIeeet1.solver.json", "dense"),
    "ThreeBusBasic": ("examples/PhasorDynamics/Tiny/ThreeBus/Basic/ThreeBusBasic.solver.json", "dense"),
    "ACTIVSg200": ("examples/PhasorDynamics/validation/ACTIVSg200/ACTIVSg200.solver.json", "dense"),
    "ACTIVSg500": ("examples/PhasorDynamics/validation/ACTIVSg500/ACTIVSg500.solver.json", "dense"),
    "WECC240": ("examples/PhasorDynamics/validation/WECC240/WECC240.solver.json", "coarse"),
    "ACTIVSg10k": ("examples/PhasorDynamics/Huge/activsg10k/activsg10k.solver.json", "coarse"),
}


def mu_tag(mu: float) -> str:
    exp = f"{mu:.10g}"
    return exp.replace(".", "p")


def make_variant(case: str, src: Path, mode: str, mu: float, tight: bool, out_root: Path) -> Path:
    with open(src) as f:
        base = json.load(f)

    # Re-anchor the case file path at the source solver.json's directory
    model = (src.parent / base["system_model_file"]).resolve()
    if not model.exists():
        raise FileNotFoundError(model)

    name = f"{mode}-mu{mu_tag(mu)}" + ("-tight" if tight else "")
    run_dir = out_root / case / name
    run_dir.mkdir(parents=True, exist_ok=True)

    cfg = dict(base)
    cfg["system_model_file"] = str(model)
    if mode == "piecewise":
        cfg["math"] = {"mode": mode, "mu": mu, "gate_mu": GATE_MU_PIN}
    else:
        cfg["math"] = {"mode": mode, "mu": mu}
    cfg["ida_stats"] = "ida_stats.json"
    cfg["ida_steps"] = "ida_steps.json"
    # Must not collide with any case's own monitor-sink file name; the app
    # reconciles the two by symlinking, and equal names produce a self-link.
    cfg["output_file"] = "study_out.csv"
    # Accuracy is judged by analyze.py against study-internal references, not
    # the shipped regression reference (whose tolerance assumes mu = 240).
    for key in ("reference_file", "error_tolerance", "error_type", "abs_err_threshold"):
        cfg.pop(key, None)
    if tight:
        ida = dict(cfg.get("ida", {}))
        ida["rel_tol"] = TIGHT_REL_TOL
        ida["abs_tol"] = TIGHT_ABS_TOL
        cfg["ida"] = ida

    with open(run_dir / f"{case}.solver.json", "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(HERE / "work"), help="matrix output root")
    parser.add_argument("--cases", nargs="*", default=list(CASES), help="subset of cases")
    parser.add_argument("--fresh", action="store_true", help="delete existing matrix first")
    args = parser.parse_args()

    out_root = Path(args.out)
    if args.fresh and out_root.exists():
        shutil.rmtree(out_root)

    manifest = []
    for case in args.cases:
        rel_src, tier = CASES[case]
        src = REPO / rel_src
        mus = MU_DENSE if tier == "dense" else MU_COARSE
        for mu in mus:
            for tight in (False, True):
                run_dir = make_variant(case, src, "smooth", mu, tight, out_root)
                manifest.append({
                    "case": case,
                    "tier": tier,
                    "mode": "smooth",
                    "mu": mu,
                    "tight": tight,
                    "dir": str(run_dir.relative_to(out_root)),
                })
        # One pinned piecewise reference arm per case
        for tight in (False, True):
            run_dir = make_variant(case, src, "piecewise", GATE_MU_PIN, tight, out_root)
            manifest.append({
                "case": case,
                "tier": tier,
                "mode": "piecewise",
                "mu": GATE_MU_PIN,
                "tight": tight,
                "dir": str(run_dir.relative_to(out_root)),
            })

    with open(out_root / "manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")
    print(f"{len(manifest)} variants under {out_root}")


if __name__ == "__main__":
    main()
