#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def select_columns(headers, required_tokens):
    matches = []
    for header in headers:
        normalized = header.lower().replace("_", "")
        if all(token in normalized for token in required_tokens):
            matches.append(header)
    if len(matches) != 3:
        raise RuntimeError(
            f"expected three columns containing {required_tokens}, found {matches}"
        )
    return matches


def rolling_rms(times, values, period=1.0 / 60.0):
    if len(times) != len(values) or not times:
        raise RuntimeError("RMS inputs must be nonempty and have matching lengths")

    square_integral = [0.0]
    for index in range(1, len(times)):
        step = times[index] - times[index - 1]
        if step <= 0.0:
            raise RuntimeError("monitor timestamps must be strictly increasing")
        square_integral.append(
            square_integral[-1]
            + 0.5
            * step
            * (values[index - 1] ** 2 + values[index] ** 2)
        )

    result = []
    first_time = times[0]
    for end, time in enumerate(times):
        window_start = time - period
        if window_start < first_time:
            result.append(math.nan)
            continue

        right = bisect.bisect_left(times, window_start, 0, end + 1)
        if right == 0 or times[right] == window_start:
            start_integral = square_integral[right]
        else:
            left = right - 1
            fraction = (window_start - times[left]) / (times[right] - times[left])
            start_value = values[left] + fraction * (values[right] - values[left])
            start_integral = square_integral[left] + 0.5 * (
                values[left] ** 2 + start_value**2
            ) * (window_start - times[left])

        result.append(math.sqrt((square_integral[end] - start_integral) / period))
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Plot the fixed-ABC IEEE 13-node projection fault run"
    )
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("emt_ieee13_fault.png"))
    parser.add_argument("--fault-on", type=float, default=0.050)
    parser.add_argument("--fault-off", type=float, default=0.100)
    args = parser.parse_args()
    if not 0.0 <= args.fault_on < args.fault_off:
        raise RuntimeError("fault times must satisfy 0 <= fault-on < fault-off")

    with args.csv_file.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if not rows or reader.fieldnames is None:
            raise RuntimeError("monitor CSV is empty")
        headers = reader.fieldnames

    time_name = headers[0]
    times = [float(row[time_name]) for row in rows]
    voltage_names = select_columns(headers, ["671", "v"])
    source_names = select_columns(headers, ["source650", "i"])
    fault_names = select_columns(headers, ["fault671", "i"])

    voltage = [[float(row[name]) for row in rows] for name in voltage_names]
    source = [[float(row[name]) for row in rows] for name in source_names]
    fault = [[float(row[name]) for row in rows] for name in fault_names]
    fault_injected = [
        [
            value if args.fault_on <= time < args.fault_off else 0.0
            for time, value in zip(times, phase_values)
        ]
        for phase_values in fault
    ]

    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    labels = ["a", "b", "c"]
    for phase, label in enumerate(labels):
        axes[0].plot(times, voltage[phase], label=f"v{label}")
        axes[1].plot(times, rolling_rms(times, voltage[phase]), label=f"v{label} RMS")
        axes[2].plot(times, source[phase], label=f"source i{label}")
        axes[3].plot(times, fault_injected[phase], label=f"fault injected i{label}")

    for axis in axes:
        axis.axvspan(args.fault_on, args.fault_off, color="tab:red", alpha=0.12)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncols=3)
    axes[0].set_ylabel("Bus 671 V")
    axes[1].set_ylabel("Bus 671 V RMS (1 cycle)")
    axes[2].set_ylabel("Source A")
    axes[3].set_ylabel("Fault injected A")
    axes[3].set_xlabel("Time [s]")
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
