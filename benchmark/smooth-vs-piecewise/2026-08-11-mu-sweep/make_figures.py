#!/usr/bin/env python3
"""Render the study figures and tables from results.csv and the run traces.

Outputs (figures/):
  primitives.png/.pdf        ramp, qramp, and the gate in both representations
  error_vs_mu.png/.pdf       trajectory error vs mu + GENSAL trajectory/deviation
  work_vs_mu.png/.pdf        solver work relative to the piecewise arm
  activsg10k_step_collapse.png/.pdf  ACTIVSg10k through the fault
  step_profiles.png/.pdf     accepted step size vs time, both arms, six cases
  solver_effort.png/.pdf     cumulative Newton iterations and per-step LTE
  h_histograms.png/.pdf      accepted-step-size distributions, both arms
  tradeoff.png/.pdf          trajectory error vs Jacobian work frontier
  wall_time.png/.pdf         wall time vs mu with the piecewise arm marked
  counters_heatmap.png/.pdf  log2 smooth/piecewise ratio of every IDA counter
  summary.md                 per-case headline table
  counters.md                full IDA counter dump per case and arm
"""

import csv
import json
import math
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import BoundaryNorm, LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402
from matplotlib.cm import ScalarMappable  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = HERE / "work"
FIGS = HERE / "figures"
FIGS.mkdir(exist_ok=True)

# Reference palette (validated defaults): smooth = blue with mu running
# light -> dark on the blue sequential ramp, piecewise = orange, everywhere.
# The counter heatmap is the one diverging context (blue <-> red, neutral mid).
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
MU_CMAP = LinearSegmentedColormap.from_list("mu", ["#86b6ef", "#2a78d6", "#0d366b"])
DIV_CMAP = LinearSegmentedColormap.from_list("div", ["#2a78d6", "#f0efec", "#e34948"])

MU_HALF_DECADES = [10.0 ** (1.0 + 0.5 * k) for k in range(7)]
MU_DECADES = [10.0, 100.0, 1000.0, 10000.0]

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": AXIS,
    "axes.linewidth": 0.8,
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "axes.titlesize": 9.5,
    "axes.labelsize": 9,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "text.color": INK,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.5,
    "font.size": 9,
    "font.family": "STIXGeneral",
    "mathtext.fontset": "stix",
    "legend.frameon": False,
    "legend.fontsize": 8,
})

CASE_LABEL = {
    "TwoBusTgov1": "TGOV1",
    "TwoBusGensal": "GENSAL",
    "TwoBusIeeet1": "IEEET1",
    "ThreeBusBasic": "GENROU control",
    "ThreeBusGenrouSat": "GENROU saturated",
    "TwoBusGensalSat": "GENSAL saturated",
    "TwoBusTgov1Bnd": "TGOV1 boundary",
    "TwoBusIeeet1Bnd": "IEEET1 boundary",
    "TwoBusGensalBnd": "GENSAL boundary",
    "ThreeBusGenrouBnd": "GENROU boundary",
    "ACTIVSg200": "ACTIVSg200",
    "ACTIVSg500": "ACTIVSg500",
    "WECC240": "WECC240",
    "ACTIVSg10k": "ACTIVSg10k",
}
# Active primitives per case, from the case parameters: every canonical case
# runs its machines unsaturated (S10 = S12 = 0), so the qramp activity there
# is the IEEET1 exciter's; only the *Sat study-local variants exercise the
# machine saturation qramp.
CASE_PRIMITIVE = {
    "TwoBusTgov1": "anti-windup gate",
    "TwoBusGensal": "exciter qramp + gates",
    "TwoBusIeeet1": "exciter qramp + gates",
    "ThreeBusBasic": "no active primitives",
    "ThreeBusGenrouSat": "machine qramp only",
    "TwoBusGensalSat": "machine + exciter qramp + gates",
    "TwoBusTgov1Bnd": "knee + Pvmax at operating point",
    "TwoBusIeeet1Bnd": "knee + Pvmax at operating point",
    "TwoBusGensalBnd": "knee + Pvmax at operating point",
    "ThreeBusGenrouBnd": "knee at operating point, no gates",
}
PROFILE_CASES = ["TwoBusTgov1", "TwoBusGensal", "TwoBusIeeet1",
                 "ACTIVSg200", "ACTIVSg500", "WECC240"]


def mu_tag(mu):
    return f"{mu:.10g}".replace(".", "p")


def mu_color(mu):
    return MU_CMAP((math.log10(mu) - 1.0) / 3.0)


def save(fig, name):
    fig.savefig(FIGS / f"{name}.png", dpi=200, bbox_inches="tight", pad_inches=0.06)
    fig.savefig(FIGS / f"{name}.pdf", bbox_inches="tight", pad_inches=0.06)
    plt.close(fig)


def load_rows():
    with open(HERE / "results.csv") as f:
        return list(csv.DictReader(f))


def fnum(r, key):
    v = r.get(key) or ""
    return float(v) if v else None


def runs(rows, case, mode, tight=False):
    out = {}
    for r in rows:
        if r["case"] == case and r["mode"] == mode and (r["tight"] == "True") == tight:
            out[float(r["mu"])] = r
    return dict(sorted(out.items()))


def fault_band(ax, t0=1.0, t1=1.1, label=True):
    ax.axvspan(t0, t1, color=GRID, alpha=0.55, lw=0, zorder=0)
    if label:
        ax.text(0.5 * (t0 + t1), 0.955, "fault", transform=ax.get_xaxis_transform(),
                ha="center", va="top", color=MUTED, fontsize=7.5)


_TRACE_CACHE = {}


def step_trace(case, name):
    """t, h, and cumulative Newton iterations/failures plus per-step LTE."""
    key = (case, name)
    if key in _TRACE_CACHE:
        return _TRACE_CACHE[key]
    p = WORK / case / name / "ida_steps.json"
    with open(p) as f:
        s = json.load(f)
    t, h, lte = [], [], []
    nli, ncf = [], []
    tot_i = tot_f = 0
    for seg in s.get("segments", []):
        for step in seg.get("steps", []):
            t.append(step["step_end_time"])
            h.append(step["last_step"])
            lte.append(step.get("lte_wrms", 0.0))
            d = step["counter_delta"]["nonlinear_solver"]
            tot_i += d["iterations"]
            tot_f += d["convergence_failures"]
            nli.append(tot_i)
            ncf.append(tot_f)
    out = tuple(np.asarray(v) for v in (t, h, lte, nli, ncf))
    _TRACE_CACHE[key] = out
    return out


# ----------------------------------------------------------------------------
# Figure 1: ramp, qramp, and the gate in both representations


def smooth_sigmoid(x, mu):
    return 0.5 * (1.0 + np.tanh(0.5 * mu * x))


def smooth_ramp(x, mu):
    a = np.abs(mu * x)
    return 0.5 * (x + a / mu) + np.log1p(np.exp(-a)) / mu


def fig_primitives():
    x = np.linspace(-0.6, 0.6, 2401)
    mus = [10.0, 100.0, 1000.0]
    fig, axes = plt.subplots(2, 3, figsize=(7.0, 4.4), layout="constrained",
                             sharex=True)
    (ax_r, ax_q, ax_g), (ax_dr, ax_dq, ax_dg) = axes

    step = (x > 0).astype(float)
    ax_r.plot(x, np.maximum(x, 0.0), color=ORANGE, lw=2.4)
    ax_q.plot(x, np.maximum(x, 0.0) ** 2, color=ORANGE, lw=2.4)
    ax_g.plot(x, step, color=ORANGE, lw=2.4)
    ax_dr.plot(x, step, color=ORANGE, lw=2.4)
    ax_dq.plot(x, 2.0 * np.maximum(x, 0.0), color=ORANGE, lw=2.4)

    for mu in mus:
        c = mu_color(mu)
        s = smooth_sigmoid(x, mu)
        with np.errstate(over="ignore"):
            ds = 0.25 * mu / np.cosh(0.5 * mu * x) ** 2
            dq = 2.0 * x * s + x * x * ds
        ax_r.plot(x, smooth_ramp(x, mu), color=c, lw=1.5)
        ax_q.plot(x, x * x * s, color=c, lw=1.5)
        ax_g.plot(x, s, color=c, lw=1.5)
        ax_dr.plot(x, s, color=c, lw=1.5)
        ax_dq.plot(x, dq, color=c, lw=1.5)
        ax_dg.plot(x, ds, color=c, lw=1.5)

    ax_dg.set_yscale("log")
    ax_dg.set_ylim(1e-2, 8e2)
    ax_dg.text(0.03, 0.05, "exact gate:\n$f' \\equiv 0$ away\nfrom the jump",
               transform=ax_dg.transAxes, ha="left", va="bottom",
               color=INK2, fontsize=7.5)

    handles = [plt.Line2D([], [], color=ORANGE, lw=2.4, label="piecewise (exact)")]
    handles += [plt.Line2D([], [], color=mu_color(mu), lw=1.5,
                           label=rf"smooth, $\mu=10^{int(math.log10(mu))}$")
                for mu in mus]
    ax_r.legend(handles=handles, loc="upper left", handlelength=1.6, fontsize=7)

    ax_r.set_title(r"ramp: $\max(x,\,0)$", loc="left")
    ax_q.set_title(r"qramp: $\max(x,\,0)^2$", loc="left")
    ax_g.set_title(r"gate: $H(x)$", loc="left")
    ax_r.set_ylabel(r"$f(x)$")
    ax_dr.set_ylabel(r"$f'(x)$")
    ax_r.set_ylim(-0.04, 0.74)
    ax_q.set_ylim(-0.02, 0.40)
    ax_g.set_ylim(-0.06, 1.12)
    ax_dr.set_ylim(-0.06, 1.12)
    ax_dq.set_ylim(-0.07, 1.32)
    for ax in (ax_dr, ax_dq, ax_dg):
        ax.set_xlabel(r"$x$")
    for ax in axes.flat:
        ax.set_xlim(-0.6, 0.6)
    save(fig, "primitives")


# ----------------------------------------------------------------------------
# Figure 2: trajectory error vs mu, with the GENSAL trajectory as the anchor


def read_mon_col(run_dir, column):
    with open(run_dir / "study_out.csv") as f:
        reader = csv.reader(f)
        header = next(reader)
        j = header.index(column)
        t, y = [], []
        for row in reader:
            t.append(float(row[0]))
            y.append(float(row[j]))
    return np.asarray(t), np.asarray(y)


def fig_error_vs_mu(rows):
    cases = ["TwoBusGensal", "TwoBusIeeet1", "TwoBusTgov1"]
    fig = plt.figure(figsize=(7.0, 7.2), layout="constrained")
    gs = fig.add_gridspec(3, 3, height_ratios=[1.0, 0.66, 0.52])
    top = [fig.add_subplot(gs[0, i]) for i in range(3)]

    for i, (ax, case) in enumerate(zip(top, cases)):
        sm = runs(rows, case, "smooth")
        pw = runs(rows, case, "piecewise")
        mus = [m for m in sm if fnum(sm[m], "ref_model_wrms") is not None]
        err = [fnum(sm[m], "ref_model_wrms") for m in mus]
        slf = [fnum(sm[m], "ref_self_wrms") for m in mus]
        ax.plot(mus, slf, color=MUTED, lw=0, marker="o", ms=1.8, zorder=2)
        ax.plot(mus, err, color=BLUE, lw=1.5, marker="o", ms=3,
                markeredgecolor=SURFACE, markeredgewidth=0.5, zorder=3)
        floor = next(fnum(r, "ref_model_wrms") for r in pw.values())
        ax.axhline(floor, color=ORANGE, lw=1.4, ls=(0, (4, 2.5)), zorder=2)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlim(8.0, 1.3e4)
        ax.set_ylim(8e-8, 2.5e-1)
        ax.set_title(f"{CASE_LABEL[case]} ({CASE_PRIMITIVE[case]})", loc="left")
        ax.set_xlabel(r"$\mu$")
        if i == 0:
            ax.set_ylabel("trajectory WRMS deviation\nfrom exact reference")
            ax.annotate("smooth vs exact", (mus[0], err[0]), xytext=(1, -14),
                        textcoords="offset points", color=INK2, fontsize=7.5,
                        va="top")
            ax.annotate("piecewise floor", (11.0, floor), xytext=(0, -3),
                        textcoords="offset points", color=INK2, fontsize=7.5,
                        va="top")
            lo = min(v for v in slf if v)
            ax.annotate("smooth integration error\n(tight self-reference)",
                        (mus[-1], lo), xytext=(-4, -12),
                        textcoords="offset points", color=MUTED, fontsize=7,
                        ha="right", va="top")
            gx = np.array([12.0, 1.0e4])
            ax.plot(gx, 1.8 / gx, color=MUTED, lw=1.0, zorder=1)
            ax.text(120, 1.8 / 120 * 1.75, r"$\propto \mu^{-1}$", color=MUTED,
                    fontsize=8, ha="left")
        else:
            ax.tick_params(labelleft=False)

    ax_t = fig.add_subplot(gs[1, :])
    ax_d = fig.add_subplot(gs[2, :], sharex=ax_t)
    ref_dir = WORK / "TwoBusGensal" / "piecewise-mu10000-tight"
    t, ref = read_mon_col(ref_dir, "Gensal_gensal_1_1_Eqp")
    win = (t >= 0.85) & (t <= 4.0)
    ax_t.plot(t[win], ref[win], color=ORANGE, lw=2.4, zorder=2,
              label="piecewise (exact reference)")
    _, pw_def = read_mon_col(WORK / "TwoBusGensal" / "piecewise-mu10000",
                             "Gensal_gensal_1_1_Eqp")
    ax_d.plot(t[win], np.abs(pw_def - ref)[win], color=ORANGE, lw=1.4,
              ls=(0, (4, 2.5)), zorder=2)
    for mu in MU_HALF_DECADES:
        d = WORK / "TwoBusGensal" / f"smooth-mu{mu_tag(mu)}"
        _, y = read_mon_col(d, "Gensal_gensal_1_1_Eqp")
        ax_t.plot(t[win], y[win], color=mu_color(mu), lw=1.2, zorder=3)
        ax_d.plot(t[win], np.abs(y - ref)[win], color=mu_color(mu), lw=1.2,
                  zorder=3)
    fault_band(ax_t)
    fault_band(ax_d, label=False)
    ax_t.set_xlim(0.85, 4.0)
    ax_t.set_ylabel(r"GENSAL $E_q'$ (pu)")
    ax_t.legend(loc="lower right", handlelength=1.6)
    ax_t.tick_params(labelbottom=False)
    ax_d.set_yscale("log")
    ax_d.set_ylim(1e-9, 3e-1)
    ax_d.set_xlabel(r"$t$ (s)")
    ax_d.set_ylabel(r"$|E_q' - E_{q,\mathrm{ref}}'|$")

    norm = BoundaryNorm([10.0 ** (0.75 + 0.5 * k) for k in range(8)],
                        ncolors=256)
    sm = ScalarMappable(norm=norm, cmap=MU_CMAP)
    cbar = fig.colorbar(sm, ax=[ax_t, ax_d], pad=0.012, aspect=26,
                        ticks=[1e1, 1e2, 1e3, 1e4])
    cbar.set_label(r"smooth sharpness $\mu$", fontsize=8.5)
    cbar.ax.set_yscale("log")
    cbar.ax.tick_params(labelsize=8)
    cbar.outline.set_visible(False)
    save(fig, "error_vs_mu")


# ----------------------------------------------------------------------------
# Figure 3: solver work as a fraction of the piecewise arm


def fig_work_vs_mu(rows):
    metrics = [("accepted_steps", "accepted steps"),
               ("jacobian_evals", "Jacobian evaluations")]
    ordered = PROFILE_CASES + ["ThreeBusGenrouSat", "TwoBusGensalSat",
                               "ThreeBusBasic"]

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.1), layout="constrained",
                             sharey=True)
    ymax = 1.0
    for ax, (key, title) in zip(axes, metrics):
        for case in ordered:
            sm = runs(rows, case, "smooth")
            pw = runs(rows, case, "piecewise")
            base = next((fnum(r, key) for r in pw.values()
                         if fnum(r, key) is not None), None)
            if base is None:
                continue
            mus = [m for m in sm if fnum(sm[m], key) is not None]
            frac = [fnum(sm[m], key) / base for m in mus]
            ymax = max(ymax, max(frac))
            if case == "WECC240":
                ax.plot(mus, frac, color=BLUE, lw=2.0, marker="o", ms=3.5,
                        markeredgecolor=SURFACE, markeredgewidth=0.5, zorder=4)
            elif case == "ThreeBusBasic":
                ax.plot(mus, frac, color=INK2, lw=1.2, ls=(0, (4, 2.5)), zorder=3)
            else:
                ax.plot(mus, frac, color=MUTED, lw=1.0, alpha=0.9, zorder=1)
        ax.set_xscale("log")
        ax.set_xlim(8.0, 1.3e4)
        ax.set_title(title, loc="left")
        ax.set_xlabel(r"$\mu$")
    axes[0].set_ylim(0, ymax * 1.08)
    axes[0].set_ylabel("smooth  ÷  piecewise")
    handles = [
        plt.Line2D([], [], color=BLUE, lw=2.0, marker="o", ms=3.5, label="WECC240"),
        plt.Line2D([], [], color=MUTED, lw=1.0,
                   label="TGOV1, GENSAL, IEEET1, ACTIVSg200/500"),
        plt.Line2D([], [], color=INK2, lw=1.2, ls=(0, (4, 2.5)),
                   label="GENROU control (exact parity)"),
    ]
    axes[0].legend(handles=handles, loc="lower right", handlelength=1.8)
    save(fig, "work_vs_mu")


# ----------------------------------------------------------------------------
# Figure 4: ACTIVSg10k through the fault


def fig_10k(rows):
    pw_row = next(iter(runs(rows, "ACTIVSg10k", "piecewise").values()))
    pw_ok = pw_row["returncode"] == "0"
    series = [("piecewise-mu10000", ORANGE, 2.2, "piecewise")] + [
        (f"smooth-mu{mu_tag(mu)}", mu_color(mu), lw, rf"smooth, $\mu=10^{e}$")
        for mu, e, lw in ((100.0, 2, 1.5), (1000.0, 3, 1.3), (10000.0, 4, 1.3))]
    fig, (ax_h, ax_f) = plt.subplots(2, 1, figsize=(7.0, 4.8), sharex=True,
                                     layout="constrained",
                                     height_ratios=[1.0, 0.72])
    fmax_top = 0
    for zo, (name, color, lw, label) in enumerate(series):
        t, h, _, _, ncf = step_trace("ACTIVSg10k", name)
        ax_h.plot(t, h, color=color, lw=lw, label=label, zorder=3 + zo)
        ax_f.plot(t, ncf, color=color, lw=lw, zorder=3 + zo)
        fmax_top = max(fmax_top, ncf[-1] if len(ncf) else 0)
        if name.startswith("piecewise") and not pw_ok:
            ax_h.plot(t[-1], h[-1], marker="x", ms=7, mew=1.8, color=ORANGE,
                      ls="none", zorder=8)
            ax_f.plot(t[-1], ncf[-1], marker="x", ms=7, mew=1.8, color=ORANGE,
                      ls="none", zorder=8)
            ax_h.annotate(f"aborts at $t={t[-1]:.3f}$ s ($h \\to h_{{\\min}}$)",
                          (t[-1], h[-1]), xytext=(1.42, 2.5e-6),
                          textcoords="data", color=INK2, fontsize=8,
                          va="center",
                          arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8,
                                          shrinkB=4))
    note = "all four runs complete 10 s" if pw_ok else "all smooth runs\ncomplete 10 s"
    ax_h.annotate(note, (1.98, 1.3e-4), ha="right", va="top", color=INK2,
                  fontsize=8)
    fault_band(ax_h)
    fault_band(ax_f, label=False)
    ax_h.set_yscale("log")
    ax_h.set_ylim(3e-8, 4e-2)
    ax_h.set_xlim(0.0, 2.0)
    ax_h.set_xticks([0.0, 0.5, 1.0, 1.5, 2.0])
    ax_h.set_ylabel("accepted step size $h$ (s)")
    ax_h.legend(loc="lower left", bbox_to_anchor=(0.05, 0.04), handlelength=1.6)
    ax_f.set_ylim(0, max(145, fmax_top * 1.12))
    ax_f.set_ylabel("cumulative Newton\nconvergence failures")
    ax_f.set_xlabel("simulation time (s)")
    save(fig, "activsg10k_step_collapse")


# ----------------------------------------------------------------------------
# Figure 5: accepted step size vs time for both arms, six cases


def fig_step_profiles():
    fig, axes = plt.subplots(2, 3, figsize=(7.0, 4.6), layout="constrained",
                             sharex=True)
    for ax, case in zip(axes.flat, PROFILE_CASES):
        t, h, _, _, _ = step_trace(case, "piecewise-mu10000")
        ax.plot(t, h, color=ORANGE, lw=1.5, zorder=3)
        t, h, _, _, _ = step_trace(case, "smooth-mu100")
        ax.plot(t, h, color=BLUE, lw=1.1, zorder=4)
        fault_band(ax, label=False)
        ax.set_yscale("log")
        ax.set_xlim(0, 10)
        ax.set_ylim(1e-7, 6e-1)
        ax.set_title(CASE_LABEL[case], loc="left")
    for ax in axes[1]:
        ax.set_xlabel(r"$t$ (s)")
    for row in axes:
        row[0].set_ylabel("accepted $h$ (s)")
    handles = [plt.Line2D([], [], color=ORANGE, lw=1.5, label="piecewise"),
               plt.Line2D([], [], color=BLUE, lw=1.1, label=r"smooth, $\mu=10^2$")]
    axes[0, 0].legend(handles=handles, loc="lower right", handlelength=1.4,
                      fontsize=7)
    save(fig, "step_profiles")


# ----------------------------------------------------------------------------
# Figure 6: cumulative Newton iterations and per-step LTE, two cases


def fig_solver_effort():
    cases = ["TwoBusGensal", "WECC240"]
    arms = [("piecewise-mu10000", ORANGE, 1.8, "piecewise"),
            ("smooth-mu100", BLUE, 1.4, r"smooth, $\mu=10^2$"),
            ("smooth-mu10000", "#0d366b", 1.2, r"smooth, $\mu=10^4$")]
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.8), layout="constrained",
                             sharex="col")
    for col, case in enumerate(cases):
        ax_n, ax_l = axes[0, col], axes[1, col]
        for name, color, lw, label in arms:
            t, _, lte, nli, _ = step_trace(case, name)
            ax_n.plot(t, nli, color=color, lw=lw, label=label, zorder=3)
            ax_l.plot(t, np.maximum(lte, 1e-12), color=color, lw=0, marker="o",
                      ms=1.6, alpha=0.6, zorder=3)
        for ax in (ax_n, ax_l):
            fault_band(ax, label=False)
            ax.set_xlim(0, 10)
        ax_n.set_title(CASE_LABEL[case], loc="left")
        ax_l.set_yscale("log")
        ax_l.set_ylim(1e-6, 3.0)
        ax_l.axhline(1.0, color=INK2, lw=0.8, ls=(0, (4, 2.5)), zorder=2)
        ax_l.set_xlabel(r"$t$ (s)")
    axes[0, 0].set_ylabel("cumulative Newton\niterations")
    axes[1, 0].set_ylabel("per-step LTE\n(WRMS, 1 = at limit)")
    axes[1, 0].annotate("error-test limit", (0.2, 1.0), xytext=(0, 3),
                        textcoords="offset points", color=INK2, fontsize=7,
                        va="bottom")
    axes[0, 0].legend(loc="upper left", handlelength=1.4, fontsize=7)
    save(fig, "solver_effort")


# ----------------------------------------------------------------------------
# Figure 7: accepted-step-size distributions


def fig_h_hist():
    fig, axes = plt.subplots(2, 3, figsize=(7.0, 4.2), layout="constrained",
                             sharex=True, sharey=False)
    bins = np.arange(-7.5, 0.25, 0.25)
    for ax, case in zip(axes.flat, PROFILE_CASES):
        for name, color, label in (("piecewise-mu10000", ORANGE, "piecewise"),
                                   ("smooth-mu100", BLUE, r"smooth, $\mu=10^2$")):
            _, h, _, _, _ = step_trace(case, name)
            counts, edges = np.histogram(np.log10(h), bins=bins)
            ax.stairs(counts, edges, fill=True, alpha=0.25, color=color)
            ax.stairs(counts, edges, color=color, lw=1.4, label=label)
            gm = math.log10(math.exp(np.mean(np.log(h[h > 0]))))
            ax.plot([gm], [0], marker="^", ms=5, color=color, clip_on=False,
                    zorder=5)
        ax.set_title(CASE_LABEL[case], loc="left")
        ax.set_xlim(-7.5, 0)
    for ax in axes[1]:
        ax.set_xlabel(r"$\log_{10}$ accepted $h$ (s)")
    for row in axes:
        row[0].set_ylabel("accepted steps")
    axes[0, 0].legend(loc="upper left", fontsize=7, handlelength=1.4)
    axes[0, 0].text(0.03, 0.60, "triangles:\ngeometric mean",
                    transform=axes[0, 0].transAxes, color=MUTED, fontsize=6.5,
                    va="top")
    save(fig, "h_histograms")


# ----------------------------------------------------------------------------
# Figure 8: accuracy-work frontier


def fig_tradeoff(rows):
    cases = ["TwoBusGensal", "TwoBusIeeet1", "TwoBusTgov1"]
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.9), layout="constrained",
                             sharey=False)
    for ax, case in zip(axes, cases):
        sm = runs(rows, case, "smooth")
        pw = next(iter(runs(rows, case, "piecewise").values()))
        mus = [m for m in sm if fnum(sm[m], "ref_model_wrms") is not None]
        err = [fnum(sm[m], "ref_model_wrms") for m in mus]
        jac = [fnum(sm[m], "jacobian_evals") for m in mus]
        ax.plot(err, jac, color=AXIS, lw=0.8, zorder=2)
        ax.scatter(err, jac, c=[mu_color(m) for m in mus], s=14, zorder=3,
                   edgecolors=SURFACE, linewidths=0.4)
        pw_err, pw_jac = fnum(pw, "ref_model_wrms"), fnum(pw, "jacobian_evals")
        ax.plot([pw_err], [pw_jac], marker="*", ms=11, color=ORANGE, ls="none",
                zorder=4, markeredgecolor=SURFACE, markeredgewidth=0.5)
        ax.set_xscale("log")
        ax.set_ylim(0, 1.18 * max(pw_jac, max(jac)))
        ax.set_title(CASE_LABEL[case], loc="left")
        ax.set_xlabel("trajectory WRMS deviation")
        if case == cases[0]:
            ax.annotate("piecewise", (pw_err, pw_jac), xytext=(8, -3),
                        textcoords="offset points", color=INK2, fontsize=7.5,
                        va="top")
    axes[0].set_ylabel("Jacobian evaluations")
    axes[0].text(0.96, 0.10, r"$\mu=10$", transform=axes[0].transAxes,
                 color="#5598e7", fontsize=7.5, ha="right")
    axes[0].text(0.35, 0.88, r"$\mu=10^4$", transform=axes[0].transAxes,
                 color="#0d366b", fontsize=7.5)
    save(fig, "tradeoff")


# ----------------------------------------------------------------------------
# Figure 9: wall time


def fig_wall(rows):
    cases = ["ACTIVSg200", "ACTIVSg500", "WECC240", "ACTIVSg10k"]
    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.6), layout="constrained")
    for ax, case in zip(axes, cases):
        sm = runs(rows, case, "smooth")
        pw = next(iter(runs(rows, case, "piecewise").values()))
        mus = [m for m in sm if fnum(sm[m], "wall_seconds") is not None
               and sm[m]["returncode"] == "0"]
        ax.plot(mus, [fnum(sm[m], "wall_seconds") for m in mus], color=BLUE,
                lw=1.6, marker="o", ms=3, markeredgecolor=SURFACE,
                markeredgewidth=0.5, zorder=3)
        w = fnum(pw, "wall_seconds")
        if pw["returncode"] == "0":
            ax.axhline(w, color=ORANGE, lw=1.4, ls=(0, (4, 2.5)), zorder=2)
        else:
            ax.axhline(w, color=ORANGE, lw=1.4, ls=(0, (1.5, 1.8)), zorder=2)
            ax.annotate("piecewise aborts", (0.5, w), xytext=(0, 3),
                        textcoords=("axes fraction", "offset points"),
                        xycoords=("axes fraction", "data"), ha="center",
                        color=INK2, fontsize=7, va="bottom")
        ax.set_xscale("log")
        ax.set_xlim(8.0, 1.3e4)
        ax.set_ylim(0, None)
        ax.set_title(CASE_LABEL[case], loc="left")
        ax.set_xlabel(r"$\mu$")
    axes[0].set_ylabel("wall time (s)")
    handles = [plt.Line2D([], [], color=BLUE, lw=1.6, marker="o", ms=3,
                          label="smooth"),
               plt.Line2D([], [], color=ORANGE, lw=1.4, ls=(0, (4, 2.5)),
                          label="piecewise")]
    axes[0].legend(handles=handles, loc="upper left", fontsize=7,
                   handlelength=1.4)
    save(fig, "wall_time")


# ----------------------------------------------------------------------------
# Figure 10: every IDA counter as a smooth/piecewise ratio heatmap


HEAT_METRICS = [
    ("accepted_steps", "steps"),
    ("residual_evals", "residual\nevals"),
    ("jacobian_evals", "Jacobian\nevals"),
    ("linear_solver_setups", "linear\nsetups"),
    ("nonlinear_iterations", "Newton\niters"),
    ("error_test_failures", "error-test\nfails"),
    ("nonlinear_convergence_failures", "Newton\nfails"),
]


def fig_counters_heatmap(rows):
    cases = PROFILE_CASES + ["ThreeBusGenrouSat", "TwoBusGensalSat",
                             "TwoBusTgov1Bnd", "TwoBusIeeet1Bnd",
                             "TwoBusGensalBnd", "ThreeBusGenrouBnd",
                             "ThreeBusBasic"]
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 4.6), layout="constrained")
    for ax, mu in zip(axes, (100.0, 10000.0)):
        m = np.full((len(cases), len(HEAT_METRICS)), np.nan)
        txt = [["" for _ in HEAT_METRICS] for _ in cases]
        for i, case in enumerate(cases):
            sm = runs(rows, case, "smooth").get(mu)
            pw = next(iter(runs(rows, case, "piecewise").values()))
            if sm is None:
                continue
            for j, (key, _) in enumerate(HEAT_METRICS):
                a, b = fnum(sm, key), fnum(pw, key)
                if a is None or b is None:
                    continue
                if a == 0 and b == 0:
                    m[i, j] = 0.0
                    txt[i][j] = "="
                elif b == 0:
                    txt[i][j] = f"{a:.0f}:0"
                elif a == 0:
                    m[i, j] = -2.5
                    txt[i][j] = f"0:{b:.0f}"
                else:
                    m[i, j] = max(-2.5, min(2.5, math.log2(a / b)))
                    txt[i][j] = f"{a / b:.2f}"
        im = ax.imshow(m, cmap=DIV_CMAP, norm=TwoSlopeNorm(0.0, -2.5, 2.5),
                       aspect="auto")
        ax.set_xticks(range(len(HEAT_METRICS)),
                      [lbl for _, lbl in HEAT_METRICS], fontsize=6.5)
        ax.set_yticks(range(len(cases)),
                      [CASE_LABEL[c] for c in cases] if ax is axes[0] else [])
        ax.set_title(rf"smooth $\mu=10^{int(math.log10(mu))}$  ÷  piecewise",
                     loc="left")
        ax.grid(False)
        for i in range(len(cases)):
            for j in range(len(HEAT_METRICS)):
                if txt[i][j]:
                    v = m[i, j]
                    dark = v is not np.nan and abs(v) > 1.4
                    ax.text(j, i, txt[i][j], ha="center", va="center",
                            fontsize=6.5,
                            color=SURFACE if dark else INK)
    cbar = fig.colorbar(im, ax=axes, pad=0.012, aspect=22,
                        ticks=[-2, -1, 0, 1, 2])
    cbar.ax.set_yticklabels(["0.25×", "0.5×", "1×", "2×", "4×"], fontsize=7.5)
    cbar.set_label("smooth ÷ piecewise", fontsize=8)
    cbar.outline.set_visible(False)
    save(fig, "counters_heatmap")


# ----------------------------------------------------------------------------
# Figure 11: the machine-saturation qramp in isolation


def read_mon(run_dir):
    with open(run_dir / "study_out.csv") as f:
        reader = csv.reader(f)
        header = next(reader)
        data = np.array([[float(x) for x in row] for row in reader])
    return header, data


def tight_model_wrms(case, mu):
    """WRMS of tight smooth(mu) vs tight piecewise: the pure model gap,
    below the default-tolerance integration floor (same formula and 1e-6
    relative floor as analyze.py)."""
    h1, a = read_mon(WORK / case / f"smooth-mu{mu_tag(mu)}-tight")
    h2, b = read_mon(WORK / case / "piecewise-mu10000-tight")
    assert h1 == h2 and a.shape == b.shape
    scale = np.maximum(np.abs(b[:, 1:]), 1.0e-6)
    e = (a[:, 1:] - b[:, 1:]) / scale
    return math.sqrt(np.mean(e * e))


QRAMP_CASES = [("ThreeBusGenrouSat", BLUE),
               ("TwoBusGensalSat", "#1baf7a"),
               ("TwoBusGensal", MUTED)]


def fig_qramp_focus(rows):
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.4), layout="constrained")
    ax_e, ax_j = axes[0]
    ax_s, ax_h = axes[1]

    for case, color in QRAMP_CASES:
        sm = runs(rows, case, "smooth")
        mus = sorted(sm)
        ax_e.plot(mus, [tight_model_wrms(case, m) for m in mus], color=color,
                  lw=1.6, marker="o", ms=3, markeredgecolor=SURFACE,
                  markeredgewidth=0.5, label=CASE_LABEL[case], zorder=3)
        pw = next(iter(runs(rows, case, "piecewise").values()))
        for ax, key in ((ax_j, "jacobian_evals"), (ax_s, "accepted_steps")):
            base = fnum(pw, key)
            ax.plot(mus, [fnum(sm[m], key) / base for m in mus], color=color,
                    lw=1.6, zorder=3)
    gx = np.array([12.0, 1.0e4])
    ax_e.plot(gx, 1.0 / gx, color=MUTED, lw=1.0, zorder=1)
    ax_e.text(200, 1.0 / 200 * 1.8, r"$\propto \mu^{-1}$", color=MUTED,
              fontsize=8)
    ax_e.set_xscale("log")
    ax_e.set_yscale("log")
    ax_e.set_title("model gap, tight vs tight reference", loc="left")
    ax_e.set_ylabel("trajectory WRMS deviation")
    ax_e.legend(loc="lower left", handlelength=1.5, fontsize=7)

    for ax, title in ((ax_j, "Jacobian evaluations"), (ax_s, "accepted steps")):
        ax.axhline(1.0, color=INK2, lw=1.0, ls=(0, (4, 2.5)), zorder=2)
        ax.set_xscale("log")
        ax.set_ylim(0, 1.45)
        ax.set_title(f"{title}, smooth ÷ piecewise", loc="left")
    ax_j.annotate("parity", (12, 1.0), xytext=(2, -5),
                  textcoords="offset points", color=INK2, fontsize=7,
                  va="top")
    for ax in (ax_e, ax_j, ax_s):
        ax.set_xlim(8.0, 1.3e4)
        ax.set_xlabel(r"$\mu$")

    t, h, _, _, _ = step_trace("ThreeBusGenrouSat", "piecewise-mu10000")
    ax_h.plot(t, h, color=ORANGE, lw=2.0, label="piecewise", zorder=3)
    t, h, _, _, _ = step_trace("ThreeBusGenrouSat", "smooth-mu100")
    ax_h.plot(t, h, color=BLUE, lw=1.2, label=r"smooth, $\mu=10^2$", zorder=4)
    fault_band(ax_h, label=False)
    ax_h.set_yscale("log")
    ax_h.set_xlim(0, 10)
    ax_h.set_title("GENROU saturated: accepted $h$", loc="left")
    ax_h.set_xlabel(r"$t$ (s)")
    ax_h.text(0.5, 0.55, "the arms step in lockstep:\nno crossing collapses,\nunlike the gate cases",
              transform=ax_h.transAxes, ha="center", color=INK2, fontsize=7.5)
    ax_h.legend(loc="lower right", handlelength=1.4, fontsize=7)
    save(fig, "qramp_focus")


# ----------------------------------------------------------------------------
# Figures 12-13: every nonsmooth point placed at the operating point


BND_CASES = [("TwoBusTgov1Bnd", BLUE),
             ("TwoBusIeeet1Bnd", "#1baf7a"),
             ("TwoBusGensalBnd", "#4a3aa7"),
             ("ThreeBusGenrouBnd", "#eda100")]


def fig_boundary_focus(rows):
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.4), layout="constrained")
    ax_e, ax_j = axes[0]
    ax_s, ax_t = axes[1]

    for case, color in BND_CASES:
        sm = runs(rows, case, "smooth")
        mus = sorted(sm)
        ax_e.plot(mus, [tight_model_wrms(case, m) for m in mus], color=color,
                  lw=1.6, marker="o", ms=3, markeredgecolor=SURFACE,
                  markeredgewidth=0.5, label=CASE_LABEL[case], zorder=3)
        pw = next(iter(runs(rows, case, "piecewise").values()))
        for ax, key in ((ax_j, "jacobian_evals"), (ax_s, "accepted_steps")):
            base = fnum(pw, key)
            ax.plot(mus, [fnum(sm[m], key) / base for m in mus], color=color,
                    lw=1.6, zorder=3)
    ax_e.set_xscale("log")
    ax_e.set_yscale("log")
    ax_e.set_title("model gap, tight vs tight reference", loc="left")
    ax_e.set_ylabel("trajectory WRMS deviation")
    ax_e.annotate("oscillation-phase dominated\n(see caveat)", (1e3, 2e-2),
                  color="#4a3aa7", fontsize=7, ha="center")
    ax_e.legend(loc="lower left", handlelength=1.5, fontsize=7)

    for ax, title in ((ax_j, "Jacobian evaluations"), (ax_s, "accepted steps")):
        ax.axhline(1.0, color=INK2, lw=1.0, ls=(0, (4, 2.5)), zorder=2)
        ax.set_xscale("log")
        ax.set_ylim(0, 1.45)
        ax.set_title(f"{title}, smooth ÷ piecewise", loc="left")
    ax_j.annotate("parity", (12, 1.0), xytext=(2, -5),
                  textcoords="offset points", color=INK2, fontsize=7, va="top")
    for ax in (ax_e, ax_j, ax_s):
        ax.set_xlim(8.0, 1.3e4)
        ax.set_xlabel(r"$\mu$")

    # The GENSAL boundary case settles onto a mu-dependent equilibrium: the
    # smoothing tail shifts the post-fault operating point itself.
    sa = 1.0071372
    ref_dir = WORK / "TwoBusGensalBnd" / "piecewise-mu10000-tight"
    t, ref = read_mon_col(ref_dir, "Gensal_gensal_1_1_Eqp")
    ax_t.plot(t, ref, color=ORANGE, lw=2.0, label="piecewise (exact)", zorder=2)
    for mu in (100.0, 10000.0):
        d = WORK / "TwoBusGensalBnd" / f"smooth-mu{mu_tag(mu)}"
        _, y = read_mon_col(d, "Gensal_gensal_1_1_Eqp")
        ax_t.plot(t, y, color=mu_color(mu), lw=1.2, zorder=3,
                  label=rf"smooth, $\mu=10^{int(math.log10(mu))}$")
    ax_t.axhline(sa, color=INK2, lw=0.9, ls=(0, (1.5, 1.8)), zorder=1)
    ax_t.annotate("saturation knee = initial $E_q'$", (14.5, sa), xytext=(0, -4),
                  textcoords="offset points", color=INK2, fontsize=7, va="top",
                  ha="center")
    fault_band(ax_t, label=False)
    ax_t.set_xlim(0, 30)
    ax_t.set_title(r"GENSAL boundary: $E_q'$ equilibrium shift", loc="left")
    ax_t.set_xlabel(r"$t$ (s)")
    ax_t.legend(loc="lower right", handlelength=1.4, fontsize=7)
    save(fig, "boundary_focus")


def fig_boundary_profiles():
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.6), layout="constrained",
                             sharex=False)
    for ax, (case, _) in zip(axes.flat, BND_CASES):
        t, h, _, _, _ = step_trace(case, "piecewise-mu10000")
        ax.plot(t, h, color=ORANGE, lw=1.5, zorder=3)
        t, h, _, _, _ = step_trace(case, "smooth-mu100")
        ax.plot(t, h, color=BLUE, lw=1.1, zorder=4)
        fault_band(ax, label=False)
        ax.set_yscale("log")
        ax.set_xlim(0, 30 if case == "TwoBusGensalBnd" else 10)
        ax.set_ylim(1e-7, 6e-1)
        ax.set_title(f"{CASE_LABEL[case]} ({CASE_PRIMITIVE[case]})", loc="left")
    for ax in axes[1]:
        ax.set_xlabel(r"$t$ (s)")
    for row in axes:
        row[0].set_ylabel("accepted $h$ (s)")
    handles = [plt.Line2D([], [], color=ORANGE, lw=1.5, label="piecewise"),
               plt.Line2D([], [], color=BLUE, lw=1.1, label=r"smooth, $\mu=10^2$")]
    axes[0, 0].legend(handles=handles, loc="lower right", handlelength=1.4,
                      fontsize=7)
    save(fig, "boundary_profiles")


# ----------------------------------------------------------------------------
# Response metrics (time-series response PLOTS are rendered by
# make_response_plots.py with the standing pdsim tooling; this module only
# computes the frequency statistics table).
#
# TwoBus/ThreeBus machines monitor absolute speed (pu), the validation cases
# monitor omega deviation, and ACTIVSg10k monitors Va at every bus, so its
# bus frequency comes from the unwrapped angle derivative,
# f = 60 + dVa/dt / (2 pi). Derivative samples adjacent to the fault
# switching instants are masked: differentiating across a discontinuity is
# meaningless.

F0 = 60.0
EVENT_TIMES = (1.0, 1.1)


def freq_matrix(case, run):
    """t and per-machine frequency in Hz from the monitored speeds/omegas."""
    path = WORK / case / run / "study_out.csv"
    with open(path) as f:
        hdr = f.readline().strip().split(",")
    speed = [i for i, h in enumerate(hdr) if h.endswith("_speed")]
    omega = [i for i, h in enumerate(hdr) if h.endswith("_omega")]
    import pandas as pd
    df = pd.read_csv(path, usecols=[0] + speed + omega, engine="c")
    t = df.iloc[:, 0].to_numpy()
    vals = df.iloc[:, 1:].to_numpy()
    F = F0 * vals if speed else F0 * (1.0 + vals)
    return t, F


def mask_events(t, x, half=0.015):
    x = x.copy()
    for te in EVENT_TIMES:
        x[(t > te - half) & (t < te + half)] = np.nan
    return x


def freq_10k(run, stride=5):
    import pandas as pd
    path = WORK / "ACTIVSg10k" / run / "study_out.csv"
    with open(path) as f:
        hdr = f.readline().strip().split(",")
    idx = [i for i, h in enumerate(hdr) if h.endswith("_Va")][::stride]
    df = pd.read_csv(path, usecols=[0] + idx, engine="c")
    t = df.iloc[:, 0].to_numpy()
    th = np.unwrap(df.iloc[:, 1:].to_numpy(), axis=0)
    f = F0 + np.gradient(th, t, axis=0) / (2.0 * np.pi)
    return t, mask_events(t, f)


def response_metrics(rows):
    """Frequency/voltage response metrics per case and arm."""
    out = ["# Response metrics",
           "",
           "Frequency metrics from the monitored machine speeds/omegas",
           "(ACTIVSg10k: median bus frequency from unwrapped Va derivatives,",
           "5-bus stride). f is the mean machine frequency where a case has",
           "several machines. max |df| vs exact compares against the exact",
           "arm on the shared monitor grid over the window both cover",
           "(ACTIVSg10k: 1.10 s). Oscillation-phase caveat cases inflate",
           "max |df| (see README).",
           "",
           "| case | arm | f min (Hz) | t@min (s) | f end (Hz) | max abs df vs exact (Hz) | window (s) |",
           "|---|---|---|---|---|---|---|"]

    def freq_of(case, run):
        if case == "ACTIVSg10k":
            t, f = freq_10k(run)
            return t, np.nanmedian(f, axis=1)
        t, F = freq_matrix(case, run)
        return t, F.mean(axis=1)

    for case in CASE_LABEL:
        arms = []
        sm = runs(rows, case, "smooth")
        for mu in MU_DECADES:
            if mu in sm and sm[mu]["returncode"] == "0":
                arms.append((rf"smooth $\mu=10^{int(math.log10(mu))}$",
                             f"smooth-mu{mu_tag(mu)}"))
        arms.append(("piecewise", "piecewise-mu10000"))
        try:
            t_ref, f_ref = freq_of(case, "piecewise-mu10000")
        except (FileNotFoundError, StopIteration):
            continue
        for i, (label, run) in enumerate(arms):
            t, f = freq_of(case, run)
            n = min(len(t), len(t_ref))
            dmax = np.nanmax(np.abs(f[:n] - f_ref[:n]))
            k = int(np.nanargmin(f))
            lead = f"| {CASE_LABEL[case]} " if i == 0 else "| "
            out.append(f"{lead}| {label} | {np.nanmin(f):.4f} | {t[k]:.2f} | "
                       f"{f[-1]:.4f} | {dmax:.2e} | {t[min(n, len(t)) - 1]:.2f} |")
    (FIGS / "response_metrics.md").write_text("\n".join(out) + "\n")


# ----------------------------------------------------------------------------
# Tables


def fmte(v):
    return f"{v:.1e}" if v is not None else "--"


def fmt0(r, k):
    v = r.get(k)
    return v if v not in (None, "") else "--"


def case_tmax(case):
    import glob as _glob
    for sj in sorted(_glob.glob(str(WORK / case / "*" / "*.solver.json"))):
        with open(sj) as f:
            return json.load(f).get("tmax")
    return None


def arm_entries(rows, case):
    entries = []
    sm = runs(rows, case, "smooth")
    for mu in MU_DECADES:
        if mu in sm:
            entries.append((rf"smooth $\mu=10^{int(math.log10(mu))}$", sm[mu]))
    pw = runs(rows, case, "piecewise")
    entries.append(("piecewise", next(iter(pw.values()))))
    return entries


def summary(rows):
    out = ["# Per-case summary",
           "",
           "Default-tolerance runs at the decade points of the mu grid.",
           "Trajectory error is the WRMS deviation from the case's",
           "tight-tolerance piecewise reference on the monitor grid",
           "(`ref_model_wrms`); the piecewise row's value is that arm's own",
           "integration-error floor. ACTIVSg200/500 and WECC240 trajectory",
           "error is oscillation-phase dominated (see README caveat). Full",
           "grid: results.csv; all counters: counters.md.",
           "",
           "| case | states | horizon | arm | steps | Jacobians | Newton fails | wall (s) | traj. error |",
           "|---|---|---|---|---|---|---|---|---|"]
    for case in CASE_LABEL:
        tmax = case_tmax(case)
        for i, (label, r) in enumerate(arm_entries(rows, case)):
            lead = f"| {CASE_LABEL[case]} | {fmt0(r, 'states')} | {tmax:g} s " \
                if i == 0 else "| | | "
            note = "" if r["returncode"] == "0" else " **fails**"
            out.append(
                f"{lead}| {label} | {fmt0(r, 'steps')} | "
                f"{fmt0(r, 'jacobian_evals')} | "
                f"{r.get('nonlinear_convergence_failures') or '0'} | "
                f"{float(r['wall_seconds']):.2f} | "
                f"{fmte(fnum(r, 'ref_model_wrms'))}{note} |")
    (FIGS / "summary.md").write_text("\n".join(out) + "\n")


COUNTER_COLS = [
    ("steps", "steps"),
    ("residual_evals", "res evals"),
    ("jacobian_evals", "Jac evals"),
    ("linear_solver_setups", "lin setups"),
    ("nonlinear_iterations", "NL iters"),
    ("nonlinear_convergence_failures", "NL fails"),
    ("error_test_failures", "ET fails"),
    ("h_geomean", "h geomean"),
    ("h_min", "h min"),
    ("lte_wrms_mean", "LTE mean"),
    ("order_mean", "order mean"),
    ("wall_seconds", "wall (s)"),
]


def counters_table(rows):
    out = ["# Full IDA counters",
           "",
           "Every counter for the default-tolerance runs at the decade mu",
           "points and the piecewise arm. h and LTE statistics come from the",
           "per-step trace (`ida_steps.json`); LTE is the WRMS local",
           "truncation error (1 = at the error-test limit).",
           ""]
    header = "| arm | " + " | ".join(lbl for _, lbl in COUNTER_COLS) + " |"
    rule = "|---" * (len(COUNTER_COLS) + 1) + "|"
    for case in CASE_LABEL:
        out += [f"## {CASE_LABEL[case]}", "", header, rule]
        for label, r in arm_entries(rows, case):
            cells = []
            for key, _ in COUNTER_COLS:
                v = fnum(r, key)
                if v is None:
                    cells.append("--")
                elif key in ("h_geomean", "h_min", "lte_wrms_mean"):
                    cells.append(f"{v:.2e}")
                elif key in ("order_mean", "wall_seconds"):
                    cells.append(f"{v:.2f}")
                else:
                    cells.append(f"{v:.0f}")
            note = "" if r["returncode"] == "0" else " **fails**"
            out.append(f"| {label}{note} | " + " | ".join(cells) + " |")
        out.append("")
    (FIGS / "counters.md").write_text("\n".join(out) + "\n")


def main():
    rows = load_rows()
    fig_primitives()
    fig_error_vs_mu(rows)
    fig_work_vs_mu(rows)
    fig_10k(rows)
    fig_step_profiles()
    fig_solver_effort()
    fig_h_hist()
    fig_tradeoff(rows)
    fig_wall(rows)
    fig_counters_heatmap(rows)
    fig_qramp_focus(rows)
    fig_boundary_focus(rows)
    fig_boundary_profiles()
    response_metrics(rows)
    summary(rows)
    counters_table(rows)
    print("figures ->", FIGS)


if __name__ == "__main__":
    main()
