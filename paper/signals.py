"""Ordered monitor/reference CSV helpers and error metrics."""

from __future__ import annotations

import csv
import json
import math
from bisect import bisect_right
from pathlib import Path


SignalGroup = list[tuple[str, list[float]]]


def _clean_cell(cell: str) -> str:
    return cell.strip().lstrip("\ufeff")


def _to_float(cell: str) -> float:
    cell = _clean_cell(cell)
    return float(cell) if cell else float("nan")


def read_series_csv(path: Path) -> tuple[list[float], SignalGroup]:
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.reader(handle)
        header = [_clean_cell(name) for name in next(reader)]
        width = len(header)
        rows: list[list[float]] = []
        for raw in reader:
            if not raw or not any(cell.strip() for cell in raw):
                continue
            values = [_to_float(cell) for cell in raw[:width]]
            values.extend([float("nan")] * (width - len(values)))
            rows.append(values)

    rows = [row for row in rows if row and math.isfinite(row[0])]
    rows.sort(key=lambda row: row[0])
    if not rows:
        raise ValueError(f"No numeric rows in {path}")

    time = [row[0] for row in rows]
    signals = [
        (name, [row[index] for row in rows])
        for index, name in enumerate(header)
        if index > 0
    ]
    return time, signals


def signal_group(signals: SignalGroup, suffix: str) -> SignalGroup:
    return [(name, values) for name, values in signals if name.endswith(suffix)]


def _interp(xs: list[float], ys: list[float], x: float) -> float:
    if x < xs[0] or x > xs[-1]:
        return float("nan")
    right = bisect_right(xs, x)
    if right == 0:
        return ys[0]
    if right >= len(xs):
        return ys[-1]
    left = right - 1
    x0, x1 = xs[left], xs[right]
    y0, y1 = ys[left], ys[right]
    if x1 == x0:
        return y1
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0))


def group_error(
    output_time: list[float],
    output_group: SignalGroup,
    reference_time: list[float],
    reference_group: SignalGroup,
    suffix: str,
) -> dict[str, float | int] | None:
    if not output_group and not reference_group:
        return None
    if len(output_group) != len(reference_group):
        raise ValueError(
            f"Mismatched {suffix} column counts: output has {len(output_group)}, "
            f"reference has {len(reference_group)}"
        )

    lo = max(output_time[0], reference_time[0])
    hi = min(output_time[-1], reference_time[-1])
    times = [t for t in output_time if lo <= t <= hi]
    if not times:
        return None

    squared_error = 0.0
    max_error = 0.0
    count = 0
    time_indices = [(index, t) for index, t in enumerate(output_time) if lo <= t <= hi]
    for (_out_name, out_values), (_ref_name, ref_values) in zip(output_group, reference_group):
        for index, t in time_indices:
            out_value = out_values[index]
            ref_value = _interp(reference_time, ref_values, t)
            if not (math.isfinite(out_value) and math.isfinite(ref_value)):
                continue
            err = ref_value - out_value
            squared_error += err * err
            max_error = max(max_error, abs(err))
            count += 1

    return {
        "rmse": math.sqrt(squared_error / count) if count else float("nan"),
        "max": max_error if count else float("nan"),
        "columns": len(output_group),
        "samples": count,
    }


def write_error_report(case_key: str, output_csv: Path, reference_csv: Path, dest: Path) -> dict:
    output_time, output_signals = read_series_csv(output_csv)
    reference_time, reference_signals = read_series_csv(reference_csv)
    report = {
        "schema": "gridkit.validation.error.v1",
        "case": case_key,
        "output": str(output_csv),
        "reference": str(reference_csv),
        "t_overlap": [max(output_time[0], reference_time[0]), min(output_time[-1], reference_time[-1])],
        "omega": group_error(
            output_time,
            signal_group(output_signals, "_omega"),
            reference_time,
            signal_group(reference_signals, "_omega"),
            "_omega",
        ),
        "vm": group_error(
            output_time,
            signal_group(output_signals, "_Vm"),
            reference_time,
            signal_group(reference_signals, "_Vm"),
            "_Vm",
        ),
    }
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
