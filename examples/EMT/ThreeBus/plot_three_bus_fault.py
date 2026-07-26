#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

# Reproduce the checked-in plots from monitor CSVs in the same directory:
#   python3 plot_three_bus_fault.py ThreeBus.csv
#   python3 plot_three_bus_fault.py Adaptive/ThreeBusAdaptive.csv \
#       --fault-on 0.0001 --fault-off 0.0002 \
#       --output Adaptive/ThreeBusAdaptive.png
#   python3 plot_three_bus_fault.py Fixed/ThreeBusFixed.csv \
#       --fault-on 0.0001 --fault-off 0.0002 --integration-mode fixed \
#       --output Fixed/ThreeBusFixed.png


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


def instantaneous_abc_rms(voltage):
    return [
        math.sqrt((va * va + vb * vb + vc * vc) / 3.0)
        for va, vb, vc in zip(*voltage)
    ]


def main():
    parser = argparse.ArgumentParser(description="Plot the EMT three-bus fault run")
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--output", type=Path, default=Path("ThreeBus.png"))
    parser.add_argument("--fault-on", type=float, default=0.050)
    parser.add_argument("--fault-off", type=float, default=0.100)
    parser.add_argument(
        "--integration-mode",
        choices=("adaptive", "fixed"),
        default="adaptive",
    )
    args = parser.parse_args()
    if not 0.0 <= args.fault_on < args.fault_off:
        raise RuntimeError("fault times must satisfy 0 <= fault-on < fault-off")

    with args.csv_file.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if len(rows) < 2 or reader.fieldnames is None:
            raise RuntimeError("monitor CSV must contain at least two data rows")
        headers = reader.fieldnames

    time_name = headers[0]
    times = [float(row[time_name]) for row in rows]
    if not all(math.isfinite(time) for time in times):
        raise RuntimeError("monitor timestamps must be finite")
    if any(end <= start for start, end in zip(times, times[1:])):
        raise RuntimeError("monitor timestamps must be strictly increasing")
    voltage_names = select_columns(headers, ["bus632v"])
    source_names = select_columns(headers, ["source650", "i"])
    fault_names = select_columns(headers, ["loadzfault632", "i"])

    voltage = [[float(row[name]) for row in rows] for name in voltage_names]
    source = [[float(row[name]) for row in rows] for name in source_names]
    fault = [[float(row[name]) for row in rows] for name in fault_names]
    if not all(
        math.isfinite(value)
        for signal in (voltage, source, fault)
        for phase_values in signal
        for value in phase_values
    ):
        raise RuntimeError("monitor CSV contains non-finite values")
    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    mode_label = "adaptive" if args.integration_mode == "adaptive" else "fixed step"
    monitor_step_us = 1.0e6 * (times[1] - times[0])
    figure.suptitle(
        "IEEE 13 reduced three-bus switched fault — "
        f"IDA {mode_label}, {monitor_step_us:g} us monitor"
    )
    labels = ["a", "b", "c"]
    for phase, label in enumerate(labels):
        axes[0].plot(times, voltage[phase], label=f"v{label}")
        axes[2].plot(times, source[phase], label=f"source i{label}")
        axes[3].plot(times, fault[phase], label=f"fault branch i{label}")

    if times[-1] - times[0] >= 1.0 / 60.0:
        for phase, label in enumerate(labels):
            axes[1].plot(
                times,
                rolling_rms(times, voltage[phase]),
                label=f"v{label} RMS",
            )
        axes[1].set_ylabel("V RMS\n(monitor-sampled 1 cycle)")
    else:
        axes[1].plot(
            times,
            instantaneous_abc_rms(voltage),
            label="instantaneous ABC RMS",
            color="black",
        )
        axes[1].set_ylabel("V ABC RMS")

    for axis in axes:
        axis.axvspan(args.fault_on, args.fault_off, color="tab:red", alpha=0.12)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncols=3)
    axes[0].set_ylabel("V")
    axes[2].set_ylabel("A")
    axes[3].set_ylabel("A")
    axes[3].set_xlabel("Time [s]")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    figure.savefig(args.output, dpi=160)
    plt.close(figure)


if __name__ == "__main__":
    main()
