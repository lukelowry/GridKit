"""Plotting helpers for validation outputs."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from cases import Case, reference_path
from diagnostics import (
    TolerancePoint,
    load_contingency_effort,
    load_step_history,
    load_tolerance_points,
)
from signals import SignalGroup, read_series_csv, signal_group
from style import (
    CASE_COLORS,
    DPI,
    GRID_COLOR,
    LINE_COLOR,
    METRIC_COLORS,
    SPINE_COLOR,
    add_quarter_cycle_reference,
    configure,
    iqr_band,
    log_upper,
    panel_label,
    style_spines,
)


PUBLICATION_STEP_Y_MIN = 1.0e-3
SIGNAL_COLOR = "#08306b"


@dataclass(frozen=True)
class SignalQuantity:
    key: str
    suffix: str
    ylabel: str
    gridkit_label: str
    reference_label: str
    y_floor: float | None = None
    y_ceil: float | None = None


SIGNAL_QUANTITIES = {
    "voltage": SignalQuantity(
        key="voltage",
        suffix="_Vm",
        ylabel=r"$|V|$ - Voltage [p.u.]",
        gridkit_label="GridKit",
        reference_label="PowerWorld",
        y_floor=0.9,
    ),
    "speed": SignalQuantity(
        key="speed",
        suffix="_omega",
        ylabel=r"$\omega$ - Speed deviation [p.u.]",
        gridkit_label="GridKit",
        reference_label="PowerWorld",
        y_ceil=0.006,
    ),
}


def _numpy():
    try:
        import numpy as np
    except ImportError as exc:
        raise SystemExit("Plotting dense validation signals requires numpy.") from exc
    return np


def _case_color(index: int) -> str:
    return CASE_COLORS[index % len(CASE_COLORS)]


def _finish(plt, fig, output: Path, show: bool, *, tight: bool = True) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if tight:
        fig.tight_layout()
    fig.savefig(output, dpi=DPI)
    if show:
        plt.show()
    plt.close(fig)
    print(f"Wrote {output}")


def _finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _positive_pairs(xs: list[float], ys: list[float | None]) -> tuple[list[float], list[float]]:
    pairs = [(x, y) for x, y in zip(xs, ys) if y is not None and y > 0.0 and math.isfinite(y)]
    return [x for x, _y in pairs], [y for _x, y in pairs]


def _histogram_bins(data):
    np = _numpy()
    edges = np.histogram_bin_edges(data, bins="auto")
    count = len(edges) - 1
    if count < 8:
        return np.histogram_bin_edges(data, bins=8)
    if count > 22:
        return np.histogram_bin_edges(data, bins=22)
    return edges


def _event_boundaries(output_root: Path, case_key: str, window: float) -> list[float]:
    history = load_step_history(output_root, case_key)
    if history is None:
        boundaries = (1.0, 1.1)
    else:
        boundaries = history.boundaries
    return [boundary for boundary in boundaries if 0.0 < boundary < window]


def _history_t_max(history) -> float:
    return max([sample.time for sample in history.samples] + history.boundaries, default=0.0)


def _padded_limits(values, lower_floor: float | None = None) -> tuple[float, float]:
    np = _numpy()
    lo = float(np.min(values))
    hi = float(np.max(values))
    if hi <= lo:
        delta = max(abs(lo) * 0.12, 1.0)
        lo -= delta
        hi += delta
    else:
        pad = 0.08 * (hi - lo)
        lo -= pad
        hi += pad
    if lower_floor is not None:
        lo = max(lower_floor, lo)
    return lo, hi


def plot_step_size(cases: list[Case], output_root: Path, output: Path, show: bool = False) -> None:
    plt = configure(show)
    from matplotlib.ticker import AutoMinorLocator, LogFormatterMathtext, LogLocator

    fig, ax = plt.subplots(figsize=(3.5, 2.35))
    all_values: list[float] = []
    t_max = 0.0
    plotted = False

    for index, case in enumerate(cases):
        history = load_step_history(output_root, case.key)
        if history is None or not history.samples:
            print(f"  missing step history for {case.key}")
            continue

        times = [sample.time for sample in history.samples]
        steps = [sample.step_size for sample in history.samples]
        color = _case_color(index)
        ax.step(
            times,
            steps,
            where="post",
            label=case.label,
            color=color,
            linewidth=1.1,
            alpha=0.95,
            solid_capstyle="butt",
            solid_joinstyle="miter",
            clip_on=True,
        )
        all_values.extend(steps)
        t_max = max(t_max, max(times))
        for boundary in history.boundaries:
            if 0.0 < boundary < max(times):
                ax.axvline(boundary, color="#2F3437", linestyle=":", linewidth=0.55, alpha=0.30, zorder=0)
        plotted = True

    if not plotted:
        raise SystemExit("No step histories found. Run the 'run --steps' command first.")

    y_min = PUBLICATION_STEP_Y_MIN
    ax.set_yscale("log")
    ax.set_ylim(y_min, log_upper(all_values, y_min))
    ax.set_xlim(0.0, t_max)
    ax.yaxis.set_major_locator(LogLocator(base=10.0))
    ax.yaxis.set_minor_locator(LogLocator(base=10.0, subs=(2.0, 5.0)))
    ax.yaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    add_quarter_cycle_reference(ax)
    ax.set_xlabel(r"$t$ - Time [sec]")
    ax.set_ylabel(r"$h$ - Time step [sec]")
    ax.grid(True, which="major", axis="y", color="#D4D8DD", linewidth=0.42, alpha=0.75)
    ax.grid(True, which="major", axis="x", color="#E6E8EB", linewidth=0.36, alpha=0.5)
    ax.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        frameon=False,
        fontsize=7.0,
        ncol=min(5, len(cases)),
        handlelength=1.3,
        columnspacing=1.2,
        handletextpad=0.5,
    )
    style_spines(ax)
    _finish(plt, fig, output, show)


def plot_contingency(
    cases: list[Case],
    output_root: Path,
    output_dir: Path,
    include_failed: bool = False,
    show: bool = False,
) -> None:
    np = _numpy()
    plt = configure(show)
    from matplotlib.ticker import MaxNLocator

    metrics = (
        ("steps", "IDA steps"),
        ("residual_evals", "Residual evaluations"),
        ("jacobian_evals", "Jacobian evaluations"),
        ("nonlinear_iterations", "Nonlinear iterations"),
    )
    wrote = False
    for case in cases:
        rows = load_contingency_effort(output_root, case.key, include_failed=include_failed)
        if not rows:
            print(f"  missing contingency summary for {case.key}")
            continue

        fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.7), constrained_layout=True)
        for index, (ax, (field, label)) in enumerate(zip(axes.flat, metrics)):
            data = np.asarray([getattr(row, field) for row in rows], dtype=float)
            color = METRIC_COLORS[index]
            ax.set_axisbelow(True)
            ax.grid(True, axis="y", color=GRID_COLOR, linewidth=0.5, alpha=0.9)
            if data.size:
                median = iqr_band(ax, data, color)
                ax.hist(
                    data,
                    bins=_histogram_bins(data),
                    color=color,
                    alpha=0.92,
                    edgecolor="white",
                    linewidth=0.5,
                    zorder=2,
                )
                ax.axvline(median, color=LINE_COLOR, linewidth=1.0, alpha=0.9, zorder=3)
                ax.set_ylim(0, ax.get_ylim()[1] * 1.12)
            ax.set_xlabel(label)
            if index % 2 == 0:
                ax.set_ylabel("Contingencies")
            ax.yaxis.set_major_locator(MaxNLocator(nbins=4, integer=True))
            ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
            ax.margins(x=0.02)
            panel_label(ax, chr(ord("a") + index))
            style_spines(ax)
        output = output_dir / f"contingency_{case.key}.png"
        _finish(plt, fig, output, show, tight=False)
        wrote = True
    if not wrote:
        raise SystemExit("No contingency figures written. Run the 'run --contingency' command first.")


def _draw_empty_square_panel(ax, letter: str, title: str) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_box_aspect(1.0)
    panel_label(ax, letter)
    ax.text(0.19, 0.95, title, transform=ax.transAxes, color=LINE_COLOR, fontsize=8.0, va="top", ha="left")
    ax.text(0.5, 0.5, "Data pending", transform=ax.transAxes, color="#747A84", fontsize=7.2,
            ha="center", va="center")
    style_spines(ax)


def plot_contingency_frontier(
    cases: list[Case],
    output_root: Path,
    output: Path,
    include_failed: bool = False,
    show: bool = False,
) -> None:
    np = _numpy()
    plt = configure(show)
    from matplotlib.ticker import MaxNLocator

    panel_data = []
    usable_count = 0
    total_faults = 0
    for case in cases:
        rows = load_contingency_effort(output_root, case.key, include_failed=include_failed)
        if rows:
            usable_count += len(rows)
            total_faults += len(rows)
        panel_data.append((case, rows))

    if usable_count == 0:
        raise SystemExit("No contingency summaries found. Run the 'run --contingency' command first.")

    rows_count, columns = _layout_shape(len(panel_data))
    figsize = (3.32, 3.5) if (rows_count, columns) == (2, 2) else (2.65 * columns, 2.65 * rows_count)
    fig, axes = plt.subplots(rows_count, columns, figsize=figsize, squeeze=False)
    axes = list(axes.flat)

    for panel_index, (case, rows) in enumerate(panel_data):
        ax = axes[panel_index]
        ax.set_box_aspect(1.0)
        ax.set_axisbelow(True)
        letter = chr(ord("a") + panel_index)
        if not rows:
            print(f"  missing contingency summary for {case.key}")
            _draw_empty_square_panel(ax, letter, case.label)
            continue

        color = _case_color(panel_index)
        steps = np.asarray([row.steps for row in rows], dtype=float)
        jacobian_rates = np.asarray([row.jacobian_evals / row.steps for row in rows if row.steps > 0], dtype=float)
        if steps.size != jacobian_rates.size or steps.size == 0:
            _draw_empty_square_panel(ax, letter, case.label)
            continue

        ax.grid(True, color=GRID_COLOR, linewidth=0.4, alpha=0.85)
        ax.scatter(
            steps,
            jacobian_rates,
            s=16.0,
            color=color,
            alpha=0.85,
            edgecolors="white",
            linewidths=0.3,
            rasterized=True,
            zorder=3,
        )
        ax.axvline(float(np.median(steps)), color=LINE_COLOR, linewidth=0.6, alpha=0.36, zorder=1)
        ax.axhline(float(np.median(jacobian_rates)), color=LINE_COLOR, linewidth=0.6, alpha=0.36, zorder=1)
        ax.set_xlim(*_padded_limits(steps))
        ax.set_ylim(*_padded_limits(jacobian_rates, lower_floor=0.0))
        ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.tick_params(labelsize=7.5, pad=1.6)
        if panel_index % columns != 0:
            ax.yaxis.tick_right()
            ax.tick_params(labelleft=False, labelright=True)
        panel_label(ax, letter)
        ax.text(0.19, 0.95, case.label, transform=ax.transAxes, color=LINE_COLOR, fontsize=8.0,
                va="top", ha="left")
        style_spines(ax)

    for ax in axes[len(panel_data):]:
        ax.set_visible(False)

    fig.supxlabel("Accepted IDA steps", fontsize=8.5, y=0.02)
    fig.supylabel("Jacobian evals / step", fontsize=8.5, x=0.012)
    fig.subplots_adjust(left=0.135, right=0.985, bottom=0.115, top=0.985, wspace=0.08, hspace=0.16)
    _finish(plt, fig, output, show, tight=False)
    print(f"  used {usable_count}/{total_faults} contingency stats")


CUMULATIVE_WORK_METRICS = (
    ("accepted_steps", "Accepted IDA steps"),
    ("residual_evals", "Residual evaluations"),
    ("jacobian_evals", "Jacobian evaluations"),
    ("error_test_failures", "Error-test failures"),
)


def _cumulative_series(history, metric: str) -> tuple[list[float], list[float]]:
    total = sum(getattr(sample, metric) for sample in history.samples)
    if total <= 0:
        return [], []
    times = [0.0]
    values = [0.0]
    cumulative = 0
    for sample in history.samples:
        cumulative += getattr(sample, metric)
        times.append(sample.time)
        values.append(cumulative / total)
    return times, values


def _shared_boundaries(histories: list[tuple[int, Case, Any]], t_max: float) -> list[float]:
    boundaries = set()
    for _case_index, _case, history in histories:
        for boundary in history.boundaries:
            if 0.0 < boundary < t_max:
                boundaries.add(round(boundary, 12))
    return sorted(boundaries)


def _metric_label(ax, letter: str, title: str, *, loc: str = "upper-left") -> None:
    panel_label(ax, letter)
    if loc == "lower-right":
        ax.text(0.97, 0.08, title, transform=ax.transAxes, color=LINE_COLOR, fontsize=8.0,
                va="bottom", ha="right")
    else:
        ax.text(0.13, 0.95, title, transform=ax.transAxes, color=LINE_COLOR, fontsize=8.0,
                va="top", ha="left")


def plot_cumulative_work(cases: list[Case], output_root: Path, output: Path, show: bool = False) -> None:
    plt = configure(show)
    from matplotlib.ticker import AutoMinorLocator, FixedLocator, PercentFormatter

    histories = []
    for case_index, case in enumerate(cases):
        history = load_step_history(output_root, case.key)
        if history is None or not history.samples:
            print(f"  missing step history for {case.key}")
            continue
        histories.append((case_index, case, history))

    if not histories:
        raise SystemExit("No step-history data found. Run the 'run --steps' command first.")

    fig, axes = plt.subplots(2, 2, figsize=(3.32, 3.78), sharex=True, sharey=True)
    axes = list(axes.flat)
    t_max = max(_history_t_max(history) for _case_index, _case, history in histories)
    boundaries = _shared_boundaries(histories, t_max)
    legend_handles = []
    legend_labels = []

    for metric_index, (metric, title) in enumerate(CUMULATIVE_WORK_METRICS):
        ax = axes[metric_index]
        ax.set_box_aspect(1.0)
        ax.set_axisbelow(True)
        ax.grid(True, which="major", axis="both", color=GRID_COLOR, linewidth=0.4, alpha=0.85)

        for boundary in boundaries:
            ax.axvline(boundary, color=LINE_COLOR, linestyle=":", linewidth=0.55, alpha=0.28, zorder=0)

        for case_index, case, history in histories:
            times, values = _cumulative_series(history, metric)
            if not times:
                continue
            line = ax.step(
                times,
                values,
                where="post",
                color=_case_color(case_index),
                linewidth=1.1,
                alpha=0.95,
                label=case.label,
            )[0]
            if metric_index == 0:
                legend_handles.append(line)
                legend_labels.append(case.label)

        ax.set_ylim(0.0, 1.0)
        _metric_label(ax, chr(ord("a") + metric_index), title, loc="lower-right")
        if metric_index >= 2:
            ax.set_xlabel(r"$t$ - Time [sec]", fontsize=8.5)
        ax.yaxis.set_major_locator(FixedLocator((0.0, 0.25, 0.5, 0.75, 1.0)))
        ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0, decimals=0))
        style_spines(ax)
        ax.tick_params(labelsize=7.5)

    for ax in axes:
        ax.set_xlim(0.0, t_max)
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))

    fig.legend(
        handles=legend_handles,
        labels=legend_labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        frameon=False,
        fontsize=7.0,
        ncol=min(5, len(histories)),
        handlelength=1.3,
        columnspacing=1.2,
        handletextpad=0.5,
    )
    fig.supylabel("Cumulative work", fontsize=8.5, x=0.012)
    fig.subplots_adjust(left=0.135, right=0.985, bottom=0.105, top=0.9, wspace=0.08, hspace=0.16)
    _finish(plt, fig, output, show, tight=False)


WORK_PER_STEP_METRICS = (
    ("residual_evals", "Residual evals / step", "median"),
    ("nonlinear_iterations", "Nonlinear iters / step", "median"),
    ("jacobian_evals", "Jacobian evals / step", "mean"),
)


def _metric_values(history, metric: str):
    np = _numpy()
    x_values = []
    y_values = []
    for sample in history.samples:
        if sample.accepted_steps <= 0 or sample.step_size <= 0.0:
            continue
        value = getattr(sample, metric) / sample.accepted_steps
        if math.isfinite(value):
            x_values.append(sample.step_size)
            y_values.append(value)
    return np.asarray(x_values, dtype=float), np.asarray(y_values, dtype=float)


def _log_edges(histories: list[tuple[int, Case, Any]], bin_count: int = 26):
    np = _numpy()
    values = [
        sample.step_size
        for _case_index, _case, history in histories
        for sample in history.samples
        if sample.step_size > 0.0
    ]
    if not values:
        return np.asarray([], dtype=float)
    return np.logspace(math.log10(min(values)), math.log10(max(values)), bin_count)


def _binned_curve(x_values, y_values, edges, reducer: str):
    np = _numpy()
    if x_values.size == 0 or edges.size < 2:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)

    centers = []
    values = []
    bin_ids = np.digitize(x_values, edges) - 1
    reduce_fn = np.mean if reducer == "mean" else np.median
    for bin_index in range(edges.size - 1):
        mask = bin_ids == bin_index
        if int(np.count_nonzero(mask)) < 3:
            continue
        centers.append(math.sqrt(edges[bin_index] * edges[bin_index + 1]))
        values.append(float(reduce_fn(y_values[mask])))
    return np.asarray(centers, dtype=float), np.asarray(values, dtype=float)


def plot_work_per_step(cases: list[Case], output_root: Path, output: Path, show: bool = False) -> None:
    plt = configure(show)
    from matplotlib.ticker import LogFormatterMathtext, LogLocator, MaxNLocator

    histories = []
    for case_index, case in enumerate(cases):
        history = load_step_history(output_root, case.key)
        if history is None or not history.samples:
            print(f"  missing step history for {case.key}")
            continue
        histories.append((case_index, case, history))

    if not histories:
        raise SystemExit("No step-history data found. Run the 'run --steps' command first.")

    edges = _log_edges(histories)
    if edges.size < 2:
        raise SystemExit("No positive accepted-step sizes found.")

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.55), sharex=True, constrained_layout=True)
    for metric_index, (metric, ylabel, reducer) in enumerate(WORK_PER_STEP_METRICS):
        ax = axes[metric_index]
        ax.set_axisbelow(True)
        ax.grid(True, which="major", axis="both", color=GRID_COLOR, linewidth=0.45, alpha=0.85)

        for case_index, case, history in histories:
            color = _case_color(case_index)
            x_values, y_values = _metric_values(history, metric)
            if x_values.size == 0:
                continue
            ax.scatter(
                x_values,
                y_values,
                s=3.0,
                color=color,
                alpha=0.055,
                linewidths=0,
                rasterized=True,
                zorder=2,
            )
            centers, binned_values = _binned_curve(x_values, y_values, edges, reducer)
            if centers.size:
                ax.plot(
                    centers,
                    binned_values,
                    color=color,
                    linewidth=1.05,
                    marker="o",
                    markersize=2.1,
                    label=case.label if metric_index == 0 else "_nolegend_",
                    zorder=4,
                )

        ax.set_xscale("log")
        ax.set_ylabel(ylabel)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
        if metric == "jacobian_evals":
            ax.set_ylim(-0.04, 1.04)
        else:
            ax.margins(y=0.08)
        panel_label(ax, chr(ord("a") + metric_index))
        style_spines(ax)

    axes[-1].set_xlabel(r"$h$ - Time step [sec]")
    axes[-1].xaxis.set_major_locator(LogLocator(base=10.0))
    axes[-1].xaxis.set_minor_locator(LogLocator(base=10.0, subs=(2.0, 5.0)))
    axes[-1].xaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
    axes[0].legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.03),
        frameon=False,
        fontsize=7.0,
        ncol=min(5, len(histories)),
        handlelength=1.3,
        columnspacing=1.1,
        handletextpad=0.5,
    )
    _finish(plt, fig, output, show, tight=False)


def _tolerance_step_files(output_root: Path, case_key: str) -> list[tuple[float, Path]]:
    pairs = []
    for path in (output_root / case_key / "tol_sweep").glob("rtol_*.ida_steps.json"):
        tag = path.name.split(".", 1)[0]
        try:
            rtol = float(tag.split("_", 1)[1])
        except (IndexError, ValueError):
            continue
        pairs.append((rtol, path))
    return sorted(pairs, key=lambda pair: pair[0], reverse=True)


def _load_step_segments(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    for segment in data.get("segments", []):
        times = []
        steps = []
        for sample in segment.get("steps", []):
            time = _finite_float(sample.get("step_end_time"))
            step = _finite_float(sample.get("last_step"))
            if time is not None and step is not None and step != 0.0:
                times.append(time)
                steps.append(abs(step))
        if times:
            yield times, steps, segment.get("start_time"), segment.get("end_time")


def _tolerance_colors(plt, count: int, cmap_name: str | None):
    if not cmap_name:
        return [_case_color(index) for index in range(count)]
    import matplotlib

    cmap = matplotlib.colormaps[cmap_name]
    if count == 1:
        return [cmap(0.5)]
    lo, hi = 0.12, 0.9
    return [cmap(lo + (hi - lo) * index / (count - 1)) for index in range(count)]


def _tolerance_label(rtol: float) -> str:
    return rf"$10^{{{int(round(math.log10(rtol)))}}}$"


def _add_tolerance_colorbar(fig, ax, colors, files) -> None:
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import BoundaryNorm, ListedColormap

    cmap = ListedColormap(colors)
    boundaries = list(range(len(colors) + 1))
    sm = ScalarMappable(cmap=cmap, norm=BoundaryNorm(boundaries, len(colors)))
    cbar = fig.colorbar(sm, ax=ax, location="top", fraction=0.05, pad=0.04, aspect=42)
    cbar.set_ticks([index + 0.5 for index in range(len(colors))])
    cbar.set_ticklabels([_tolerance_label(rtol) for rtol, _path in files])
    cbar.ax.tick_params(length=0, labelsize=7.0, pad=2.0)
    cbar.outline.set_linewidth(0.6)
    cbar.outline.set_edgecolor(SPINE_COLOR)
    cbar.set_label(r"$r_{tol}$", fontsize=8.5, labelpad=2.5)


def plot_tolerance_step_size(
    cases: list[Case],
    output_root: Path,
    output_dir: Path,
    output: Path | None = None,
    show: bool = False,
    cmap: str | None = None,
) -> None:
    if output is not None and len(cases) != 1:
        raise SystemExit("--output can only be used with one case for tol-step-size")

    plt = configure(show)
    from matplotlib.ticker import AutoMinorLocator, LogFormatterMathtext, LogLocator

    wrote = False
    for case in cases:
        files = _tolerance_step_files(output_root, case.key)
        if not files:
            print(f"  missing tolerance step histories for {case.key}")
            continue

        fig, ax = plt.subplots(figsize=(3.5, 2.35))
        colors = _tolerance_colors(plt, len(files), cmap)
        all_values = []
        t_max = 0.0
        for color, (_rtol, path) in zip(colors, files):
            for times, steps, seg_start, seg_end in _load_step_segments(path):
                ax.step(
                    times,
                    steps,
                    where="post",
                    color=color,
                    linewidth=1.1,
                    alpha=0.95,
                    solid_capstyle="butt",
                    solid_joinstyle="miter",
                    clip_on=True,
                )
                all_values.extend(steps)
                t_max = max(t_max, max(times))
                for boundary in (seg_start, seg_end):
                    boundary = _finite_float(boundary)
                    if boundary is not None and 0.0 < boundary < t_max + 1.0:
                        ax.axvline(boundary, color="#2F3437", linestyle=":", linewidth=0.55, alpha=0.30, zorder=0)

        ax.set_yscale("log")
        ax.set_ylim(PUBLICATION_STEP_Y_MIN, log_upper(all_values, PUBLICATION_STEP_Y_MIN))
        ax.set_xlim(0.0, t_max)
        ax.yaxis.set_major_locator(LogLocator(base=10.0))
        ax.yaxis.set_minor_locator(LogLocator(base=10.0, subs=(2.0, 5.0)))
        ax.yaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        add_quarter_cycle_reference(ax)
        ax.set_xlabel(r"$t$ - Time [sec]")
        ax.set_ylabel(r"$h$ - Time step [sec]")
        ax.grid(True, which="major", axis="y", color="#D4D8DD", linewidth=0.42, alpha=0.75)
        ax.grid(True, which="major", axis="x", color="#E6E8EB", linewidth=0.36, alpha=0.5)
        style_spines(ax)
        _add_tolerance_colorbar(fig, ax, colors, files)

        target = output or (output_dir / f"step_size_tol_{case.key}.png")
        _finish(plt, fig, target, show)
        wrote = True

    if not wrote:
        raise SystemExit("No tolerance step-size figures written. Run the 'tol-sweep' command first.")


TOLERANCE_COST_SERIES = (
    ("Accepted steps", "steps", METRIC_COLORS[0]),
    ("Residual evals", "residual_evals", METRIC_COLORS[1]),
    ("Jacobian evals", "jacobian_evals", METRIC_COLORS[2]),
    ("Wall-clock", "wall_clock_seconds", METRIC_COLORS[3]),
)


def _layout_shape(count: int) -> tuple[int, int]:
    if count <= 1:
        return 1, 1
    if count <= 4:
        return 2, 2
    columns = 3
    return math.ceil(count / columns), columns


def _normalized_cost(points: list[TolerancePoint], attr: str):
    rtols: list[float] = []
    values: list[float] = []
    for point in points:
        value = getattr(point, attr)
        if value is None or value <= 0.0:
            continue
        rtols.append(point.rel_tol)
        values.append(float(value))
    if len(values) < 2:
        return None
    baseline = values[0]
    if baseline <= 0.0:
        return None
    return rtols, [value / baseline for value in values]


def _shared_cost_ylim(datasets: list[list[TolerancePoint]]) -> tuple[float, float]:
    top = 1.0
    for points in datasets:
        for _name, attr, _color in TOLERANCE_COST_SERIES:
            series = _normalized_cost(points, attr)
            if series is not None:
                top = max(top, max(series[1]))
    upper = 10.0 ** math.ceil(math.log10(top * 1.1))
    return 0.7, max(upper, 10.0)


def _draw_tolerance_panel_title(ax, letter: str, title: str) -> None:
    panel_label(ax, letter)
    ax.text(0.13, 0.95, title, transform=ax.transAxes, color=LINE_COLOR, fontsize=8.0, va="top", ha="left")


def _draw_empty_tolerance_panel(ax, case: Case, panel_index: int) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_box_aspect(1.0)
    _draw_tolerance_panel_title(ax, chr(ord("a") + panel_index), case.label)
    ax.text(0.5, 0.5, "Data pending", transform=ax.transAxes, color="#747A84", fontsize=7.2,
            ha="center", va="center")
    style_spines(ax)


def _show_xlabels(index: int, panel_data: list[tuple[Case, list[TolerancePoint]]], columns: int) -> bool:
    below = index + columns
    return below >= len(panel_data) or not panel_data[below][1]


def _tolerance_major_ticks(datasets: list[list[TolerancePoint]]) -> list[float]:
    values = sorted({point.rel_tol for points in datasets for point in points})
    if len(values) <= 6:
        return values
    return [values[0], values[len(values) // 2], values[-1]]


def _legend_handles(plt):
    from matplotlib.lines import Line2D

    return [
        Line2D([], [], color=color, marker="o", markersize=3.4, linewidth=1.05, label=name)
        for name, _attr, color in TOLERANCE_COST_SERIES
    ]


def plot_tolerance(cases: list[Case], output_root: Path, output: Path, show: bool = False) -> None:
    panel_data = [(case, load_tolerance_points(output_root, case.key)) for case in cases]
    usable = [points for _case, points in panel_data if points]
    if not usable:
        raise SystemExit("No tolerance sweep data found. Run the 'tol-sweep' command first.")

    plt = configure(show)
    from matplotlib.ticker import FixedLocator, LogFormatterMathtext, LogLocator, NullFormatter

    rows, columns = _layout_shape(len(panel_data))
    figsize = (3.32, 3.62) if (rows, columns) == (2, 2) else (2.75 * columns, 2.75 * rows)
    fig, axes = plt.subplots(rows, columns, figsize=figsize, squeeze=False)
    axes = list(axes.flat)
    cost_ylim = _shared_cost_ylim(usable)

    for panel_index, (case, points) in enumerate(panel_data):
        ax = axes[panel_index]
        if not points:
            print(f"  missing tolerance sweep for {case.key}")
            _draw_empty_tolerance_panel(ax, case, panel_index)
            continue

        ax.set_box_aspect(1.0)
        ax.set_axisbelow(True)
        ax.set_xscale("log")
        ax.set_yscale("log")
        for _name, attr, color in TOLERANCE_COST_SERIES:
            series = _normalized_cost(points, attr)
            if series is None:
                continue
            rtols, values = series
            ax.plot(
                rtols,
                values,
                color=color,
                marker="o",
                markersize=3.4,
                linewidth=1.05,
                markeredgecolor="white",
                markeredgewidth=0.3,
                zorder=3,
            )

        ax.set_ylim(*cost_ylim)
        ax.invert_xaxis()
        ax.xaxis.set_major_locator(FixedLocator(_tolerance_major_ticks(usable)))
        ax.xaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
        ax.xaxis.set_minor_locator(LogLocator(base=10.0, subs=(1.0,), numticks=12))
        ax.xaxis.set_minor_formatter(NullFormatter())
        ax.yaxis.set_major_locator(LogLocator(base=10.0))
        ax.yaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
        ax.yaxis.set_minor_locator(LogLocator(base=10.0, subs=(2.0, 5.0)))
        ax.yaxis.set_minor_formatter(NullFormatter())
        ax.grid(True, which="major", color=GRID_COLOR, linewidth=0.4, alpha=0.85)
        ax.grid(True, which="minor", color=GRID_COLOR, linewidth=0.24, alpha=0.45)
        ax.tick_params(
            labelsize=7.5,
            pad=1.6,
            labelleft=panel_index % columns == 0,
            labelbottom=_show_xlabels(panel_index, panel_data, columns),
        )
        _draw_tolerance_panel_title(ax, chr(ord("a") + panel_index), case.label)
        style_spines(ax)

    for ax in axes[len(panel_data):]:
        ax.set_visible(False)

    fig.legend(
        handles=_legend_handles(plt),
        loc="upper center",
        bbox_to_anchor=(0.5, 0.99),
        frameon=False,
        fontsize=7.5,
        ncol=4,
        handlelength=1.5,
        columnspacing=1.25,
        handletextpad=0.55,
    )
    fig.supxlabel(r"Relative tolerance $r_{tol}$ (loose $\rightarrow$ tight)", fontsize=8.5, y=0.015)
    fig.supylabel("Solver cost, normalized to loosest tolerance", fontsize=8.5, x=0.012)
    fig.subplots_adjust(left=0.10, right=0.98, bottom=0.10, top=0.90, wspace=0.14, hspace=0.16)
    _finish(plt, fig, output, show, tight=False)


def _mu_points(summary_path: Path) -> list[tuple[float, dict[str, Any] | None]]:
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    points: list[tuple[float, dict[str, Any] | None]] = []
    for point in summary.get("points", []):
        mu = _finite_float(point.get("mu"))
        if mu is None or mu <= 0.0:
            continue
        error = point.get("error")
        points.append((mu, error if isinstance(error, dict) else None))
    points.sort(key=lambda item: item[0])
    return points


def _error_series(points: list[tuple[float, dict[str, Any] | None]], group: str, measure: str):
    mus: list[float] = []
    values: list[float] = []
    for mu, error in points:
        block = error.get(group) if isinstance(error, dict) else None
        value = _finite_float(block.get(measure)) if isinstance(block, dict) else None
        if value is not None and value > 0.0:
            mus.append(mu)
            values.append(value)
    return mus, values


def _draw_no_error(ax) -> None:
    ax.text(0.5, 0.5, "No reference data", transform=ax.transAxes, color="#747A84",
            fontsize=8.0, ha="center", va="center")


def _plot_error_panel(ax, points: list[tuple[float, dict[str, Any] | None]], measure: str,
                      ylabel: str, label_suffix: str) -> bool:
    omega_mu, omega_error = _error_series(points, "omega", measure)
    vm_mu, vm_error = _error_series(points, "vm", measure)
    if omega_error:
        ax.plot(
            omega_mu,
            omega_error,
            color=METRIC_COLORS[0],
            marker="o",
            markersize=3.2,
            linewidth=1.05,
            markeredgecolor="white",
            markeredgewidth=0.35,
            label=rf"$\omega$ {label_suffix}",
        )
    if vm_error:
        ax.plot(
            vm_mu,
            vm_error,
            color=METRIC_COLORS[1],
            marker="s",
            markersize=3.0,
            linewidth=1.05,
            markeredgecolor="white",
            markeredgewidth=0.35,
            label=rf"$|V|$ {label_suffix}",
        )
    ax.set_ylabel(ylabel)
    if omega_error or vm_error:
        ax.set_yscale("log")
        ax.legend(loc="best", frameon=False, fontsize=7.0, handlelength=1.3)
        return True
    _draw_no_error(ax)
    return False


def plot_mu(cases: list[Case], output_root: Path, output_dir: Path, show: bool = False) -> None:
    plt = configure(show)
    from matplotlib.ticker import LogFormatterMathtext, LogLocator, NullFormatter

    wrote = False
    for case in cases:
        summary_paths = sorted((output_root / case.key / "mu_sweep").glob("*/summary.json"))
        if not summary_paths:
            print(f"  missing mu sweep for {case.key}")
            continue
        for summary_path in summary_paths:
            points = _mu_points(summary_path)
            if not points:
                print(f"  no usable mu sweep points for {case.key}: {summary_path}")
                continue

            fig, (ax_max, ax_rmse) = plt.subplots(
                2,
                1,
                figsize=(3.5, 3.0),
                sharex=True,
                gridspec_kw={"hspace": 0.12},
                constrained_layout=True,
            )
            _plot_error_panel(ax_max, points, "max", "Max error [p.u.]", "max")
            _plot_error_panel(ax_rmse, points, "rmse", "RMSE [p.u.]", "RMSE")
            ax_rmse.set_xlabel(r"$\mu$ - Smoothness parameter")

            for index, ax in enumerate((ax_max, ax_rmse)):
                ax.set_xscale("log")
                ax.set_axisbelow(True)
                ax.grid(True, which="major", axis="both", color=GRID_COLOR, linewidth=0.42, alpha=0.78)
                ax.grid(True, which="minor", axis="both", color=GRID_COLOR, linewidth=0.26, alpha=0.38)
                ax.xaxis.set_major_locator(LogLocator(base=10.0))
                ax.xaxis.set_minor_locator(LogLocator(base=10.0, subs=tuple(range(2, 10))))
                ax.xaxis.set_major_formatter(LogFormatterMathtext(base=10.0))
                ax.xaxis.set_minor_formatter(NullFormatter())
                panel_label(ax, chr(ord("a") + index))
                style_spines(ax)

            output = output_dir / f"mu_{case.key}_{summary_path.parent.name}.png"
            _finish(plt, fig, output, show, tight=False)
            wrote = True
    if not wrote:
        raise SystemExit("No mu sweep figures written. Run the 'mu-sweep' command first.")


def _restrict_time(time: list[float], group: SignalGroup, upper: float) -> tuple[list[float], SignalGroup]:
    keep = [index for index, value in enumerate(time) if value <= upper]
    clipped_time = [time[index] for index in keep]
    clipped_group = [(name, [values[index] for index in keep]) for name, values in group]
    return clipped_time, clipped_group


def _add_traces(ax, time: list[float], traces: SignalGroup, color: str) -> tuple[float, float]:
    np = _numpy()
    from matplotlib.collections import LineCollection

    time_array = np.asarray(time, dtype=float)
    segments = []
    lo = math.inf
    hi = -math.inf
    for _name, values in traces:
        values_array = np.asarray(values, dtype=float)
        if values_array.size != time_array.size:
            continue
        mask = np.isfinite(time_array) & np.isfinite(values_array)
        if not mask.any():
            continue
        x = time_array[mask]
        y = values_array[mask]
        segments.append(np.column_stack([x, y]))
        lo = min(lo, float(np.nanmin(y)))
        hi = max(hi, float(np.nanmax(y)))
    if not segments:
        raise ValueError("No finite signal traces to plot")
    collection = LineCollection(
        segments,
        colors=color,
        linewidths=0.28,
        alpha=0.10,
        capstyle="round",
        rasterized=True,
    )
    ax.add_collection(collection)
    return lo, hi


def _signal_limits(quantity: SignalQuantity, lo: float, hi: float) -> tuple[float, float]:
    span = hi - lo
    pad = 0.04 * span if span > 0.0 else max(1.0e-3, 0.04 * abs(hi))
    lower = lo - pad
    upper = hi + pad
    if quantity.y_floor is not None:
        lower = min(quantity.y_floor, lower)
    if quantity.y_ceil is not None:
        upper = max(quantity.y_ceil, upper)
    return lower, upper


def plot_signals(
    cases: list[Case],
    output_root: Path,
    output_dir: Path,
    quantity: str,
    show: bool = False,
) -> None:
    plt = configure(show)
    from matplotlib.ticker import AutoMinorLocator

    signal_quantity = SIGNAL_QUANTITIES[quantity]
    wrote = False
    for case in cases:
        output_csv = output_root / case.key / f"{case.key}.csv"
        ref_csv = reference_path(case)
        if ref_csv is None or not output_csv.exists():
            print(f"  missing output/reference for {case.key}")
            continue

        t_out, out_signals = read_series_csv(output_csv)
        t_ref, ref_signals = read_series_csv(ref_csv)
        out_group = signal_group(out_signals, signal_quantity.suffix)
        ref_group = signal_group(ref_signals, signal_quantity.suffix)
        if not out_group or not ref_group:
            print(f"  no {signal_quantity.suffix} signals for {case.key}")
            continue

        window = min(max(t_out), max(t_ref))
        t_out, out_group = _restrict_time(t_out, out_group, window)
        t_ref, ref_group = _restrict_time(t_ref, ref_group, window)

        fig, (ax_top, ax_bot) = plt.subplots(
            2,
            1,
            figsize=(3.5, 3.0),
            sharex=True,
            sharey=True,
            gridspec_kw={"hspace": 0.12},
            constrained_layout=True,
        )

        lo_out, hi_out = _add_traces(ax_top, t_out, out_group, SIGNAL_COLOR)
        lo_ref, hi_ref = _add_traces(ax_bot, t_ref, ref_group, SIGNAL_COLOR)
        y_lower, y_upper = _signal_limits(signal_quantity, min(lo_out, lo_ref), max(hi_out, hi_ref))
        boundaries = _event_boundaries(output_root, case.key, window)

        for ax, letter, name in (
            (ax_top, "a", signal_quantity.gridkit_label),
            (ax_bot, "b", signal_quantity.reference_label),
        ):
            ax.set_xlim(0.0, window)
            ax.set_ylim(y_lower, y_upper)
            ax.xaxis.set_minor_locator(AutoMinorLocator(2))
            ax.yaxis.set_minor_locator(AutoMinorLocator(2))
            ax.set_ylabel(signal_quantity.ylabel)
            ax.grid(True, which="major", axis="y", color="#D4D8DD", linewidth=0.42, alpha=0.75)
            ax.grid(True, which="major", axis="x", color="#E6E8EB", linewidth=0.36, alpha=0.5)
            for boundary in boundaries:
                ax.axvline(boundary, color="#2F3437", linestyle=":", linewidth=0.55, alpha=0.30, zorder=0)
            style_spines(ax)
            panel_label(ax, letter)
            ax.text(0.975, 0.93, name, transform=ax.transAxes, fontsize=8.0,
                    fontweight="bold", va="top", ha="right")

        ax_bot.set_xlabel(r"$t$ - Time [sec]")
        output = output_dir / f"signals_{quantity}_{case.key}.png"
        _finish(plt, fig, output, show, tight=False)
        wrote = True
    if not wrote:
        raise SystemExit("No signal figures written. Run 'run --steps --output-csv' first.")
