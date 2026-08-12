#!/usr/bin/env python3
"""Render the study figures from results.csv and the ida_steps traces."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
FIGS = HERE / "figures"
FIGS.mkdir(exist_ok=True)

SURFACE = "#fcfcfb"
TEXT = "#1a1a19"
TEXT2 = "#5f5e56"
GRID = "#e5e4df"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": TEXT,
    "text.color": TEXT,
    "xtick.color": TEXT2,
    "ytick.color": TEXT2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "font.size": 10,
    "axes.titlesize": 11,
    "legend.frameon": False,
})


def load_rows():
    with open(HERE / "results.csv") as f:
        return list(csv.DictReader(f))


def get(rows, case, mode, tight):
    out = {}
    for r in rows:
        if r["case"] == case and r["mode"] == mode and (r["tight"] == "True") == tight:
            out[float(r["mu"])] = r
    return dict(sorted(out.items()))


def fnum(r, key):
    v = r.get(key) or ""
    return float(v) if v else None


def fig_error_vs_mu(rows):
    cases = [("TwoBusTgov1", BLUE), ("TwoBusGensal", ORANGE), ("TwoBusIeeet1", AQUA)]
    fig, ax = plt.subplots(figsize=(6.4, 4.2), dpi=150)
    for case, color in cases:
        sm = get(rows, case, "smooth", False)
        pw = get(rows, case, "piecewise", False)
        mus = [m for m in sm if fnum(sm[m], "ref_model_wrms") is not None]
        ax.plot(mus, [fnum(sm[m], "ref_model_wrms") for m in mus],
                color=color, lw=2, marker="o", ms=7, label=f"{case} smooth")
        floor = next((fnum(r, "ref_model_wrms") for r in pw.values()
                      if fnum(r, "ref_model_wrms") is not None), None)
        if floor is not None:
            ax.axhline(floor, color=color, lw=2, ls="--", alpha=0.55,
                       label=f"{case} piecewise floor")
        ax.annotate(case, (mus[-1], fnum(sm[mus[-1]], "ref_model_wrms")),
                    xytext=(6, 4), textcoords="offset points", color=color, fontsize=9)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("mu (smooth sharpness scale)")
    ax.set_ylabel("trajectory WRMS error vs pinned piecewise reference")
    ax.set_title("Smoothing error decays ~1/mu toward the exact model;\ndashed = the piecewise arm's own integration-error floor")
    ax.legend(fontsize=8, ncols=2)
    fig.tight_layout()
    fig.savefig(FIGS / "error_vs_mu.png")
    plt.close(fig)


def fig_work_vs_mu(rows):
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.0), dpi=150, sharex=True)
    for ax, case in zip(axes, ("TwoBusGensal", "WECC240")):
        d = get(rows, case, "smooth", False)
        mus = [m for m in d if fnum(d[m], "jacobian_evals") is not None]
        ax.plot(mus, [fnum(d[m], "jacobian_evals") for m in mus],
                color=BLUE, lw=2, marker="o", ms=7, label="smooth")
        ax.annotate("smooth", (mus[-1], fnum(d[mus[-1]], "jacobian_evals")),
                    xytext=(6, 0), textcoords="offset points", color=BLUE, fontsize=9)
        pw = get(rows, case, "piecewise", False)
        floor = next((fnum(r, "jacobian_evals") for r in pw.values()
                      if fnum(r, "jacobian_evals") is not None), None)
        if floor is not None:
            ax.axhline(floor, color=ORANGE, lw=2, ls="--",
                       label="piecewise (pinned)")
        ax.set_xscale("log")
        ax.set_title(case)
        ax.set_xlabel("mu (smooth)")
    axes[0].set_ylabel("Jacobian evaluations (10 s simulation)")
    axes[0].legend(fontsize=9)
    fig.suptitle("Jacobian work: smooth swept in mu vs the pinned piecewise arm (dashed)", y=1.0)
    fig.tight_layout()
    fig.savefig(FIGS / "work_vs_mu.png")
    plt.close(fig)


def fig_10k_collapse():
    fig, ax = plt.subplots(figsize=(7.2, 4.2), dpi=150)
    for name, color, label in (("smooth-mu100", BLUE, "smooth mu=100 (completes)"),
                               ("piecewise-mu10000", ORANGE, "piecewise (fails)")):
        p = HERE / "work" / "ACTIVSg10k" / name / "ida_steps.json"
        with open(p) as f:
            s = json.load(f)
        t, h = [], []
        for seg in s.get("segments", []):
            for step in seg.get("steps", []):
                t.append(step["step_end_time"])
                h.append(step["last_step"])
        ax.plot(t, h, color=color, lw=2, label=label)
        ax.annotate(label, (t[-1], h[-1]), xytext=(8, 0),
                    textcoords="offset points", color=color, fontsize=9)
    ax.axvspan(1.0, 1.1, color=GRID, alpha=0.5, lw=0)
    ax.text(1.05, ax.get_ylim()[1], "fault", ha="center", va="top",
            color=TEXT2, fontsize=9)
    ax.set_yscale("log")
    ax.set_xlim(0.0, 2.0)
    ax.set_xlabel("simulation time (s)")
    ax.set_ylabel("accepted step size h (s)")
    ax.set_title("ACTIVSg10k: step size through the fault")
    ax.legend(fontsize=9, loc="lower right")
    fig.tight_layout()
    fig.savefig(FIGS / "activsg10k_step_collapse.png")
    plt.close(fig)


def main():
    rows = load_rows()
    fig_error_vs_mu(rows)
    fig_work_vs_mu(rows)
    fig_10k_collapse()
    print("figures ->", FIGS)


if __name__ == "__main__":
    main()
