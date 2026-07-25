#!/usr/bin/env python3

import argparse
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
    result = []
    start = 0
    square_sum = 0.0
    for end, time in enumerate(times):
        square_sum += values[end] * values[end]
        while start < end and times[start] < time - period:
            square_sum -= values[start] * values[start]
            start += 1
        result.append(math.sqrt(square_sum / (end - start + 1)))
    return result


def main():
    parser = argparse.ArgumentParser(description="Plot the EMT three-bus fault run")
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("emt_three_bus_fault.png"))
    args = parser.parse_args()

    with args.csv_file.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if not rows or reader.fieldnames is None:
            raise RuntimeError("monitor CSV is empty")
        headers = reader.fieldnames

    time_name = headers[0]
    times = [float(row[time_name]) for row in rows]
    voltage_names = select_columns(headers, ["632", "v"])
    source_names = select_columns(headers, ["source650", "i"])
    fault_names = select_columns(headers, ["fault632", "i"])

    voltage = [[float(row[name]) for row in rows] for name in voltage_names]
    source = [[float(row[name]) for row in rows] for name in source_names]
    fault = [[float(row[name]) for row in rows] for name in fault_names]

    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    labels = ["a", "b", "c"]
    for phase, label in enumerate(labels):
        axes[0].plot(times, voltage[phase], label=f"v{label}")
        axes[1].plot(times, rolling_rms(times, voltage[phase]), label=f"v{label} RMS")
        axes[2].plot(times, source[phase], label=f"source i{label}")
        axes[3].plot(times, fault[phase], label=f"fault i{label}")

    for axis in axes:
        axis.axvspan(0.05, 0.10, color="tab:red", alpha=0.12)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncols=3)
    axes[0].set_ylabel("V")
    axes[1].set_ylabel("V RMS")
    axes[2].set_ylabel("A")
    axes[3].set_ylabel("A")
    axes[3].set_xlabel("Time [s]")
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
