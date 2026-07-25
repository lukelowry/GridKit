#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


PHASES = ("a", "b", "c")
PERIOD = 1.0 / 60.0


def read_csv(csv_file):
    with csv_file.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if len(rows) < 2 or reader.fieldnames is None:
            raise RuntimeError("monitor CSV must contain at least two data rows")
        headers = reader.fieldnames

    if headers[0] != "t":
        raise RuntimeError("monitor CSV must begin with a t column")
    if len(headers) != len(set(headers)):
        raise RuntimeError("monitor CSV contains duplicate column names")

    times = [float(row["t"]) for row in rows]
    if not all(math.isfinite(time) for time in times):
        raise RuntimeError("monitor timestamps must be finite")
    if any(end <= start for start, end in zip(times, times[1:])):
        raise RuntimeError("monitor timestamps must be strictly increasing")
    return headers, rows, times


def read_phase_values(headers, rows, prefix):
    names = [f"{prefix}{phase}" for phase in PHASES]
    missing = [name for name in names if name not in headers]
    if missing:
        raise RuntimeError(f"monitor CSV is missing columns: {missing}")

    values = [[float(row[name]) for row in rows] for name in names]
    if not all(math.isfinite(value) for phase in values for value in phase):
        raise RuntimeError(f"monitor CSV contains non-finite values for {prefix}")
    return values


def rolling_rms(times, values, period=PERIOD):
    if len(times) != len(values) or not times:
        raise RuntimeError("RMS inputs must be nonempty and have matching lengths")

    square_integral = [0.0]
    for index in range(1, len(times)):
        step = times[index] - times[index - 1]
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


def study_configuration(study):
    if study == "steady":
        return {
            "title": "Balanced RLC three-bus steady state",
            "voltage_prefix": "Bus_load_v",
            "voltage_label": "Load-bus voltage",
            "current_prefix": "LoadZ_load_3_i",
            "current_label": "Load current into bus",
            "event_label": None,
            "event_color": None,
        }
    if study == "fault":
        return {
            "title": "Balanced RLC gated three-phase fault-load injection",
            "voltage_prefix": "Bus_mid_v",
            "voltage_label": "Fault-bus voltage",
            "current_prefix": "LoadZ_fault_2_i",
            "current_label": "Fault current into bus",
            "event_label": "Three-phase fault active",
            "event_color": "tab:red",
        }
    if study == "slg_fault":
        return {
            "title": "Balanced RLC gated phase-a fault-load injection",
            "voltage_prefix": "Bus_mid_v",
            "voltage_label": "Fault-bus voltage",
            "current_prefix": "LoadZ_phase_a_ground_fault_2_i",
            "current_label": "SLG-fault current into bus",
            "event_label": "Phase-a fault active",
            "event_color": "tab:red",
        }
    if study == "load_rejection":
        return {
            "title": "Balanced RLC load rejection",
            "voltage_prefix": "Bus_load_v",
            "voltage_label": "Load-bus voltage",
            "current_prefix": "LoadZ_load_3_i",
            "current_label": "Load current into bus",
            "event_label": "Load rejected",
            "event_color": "tab:orange",
        }
    raise RuntimeError(f"unsupported study: {study}")


def gate_injected_current(study, times, values, event_on, event_off):
    gated = []
    for phase_values in values:
        injected = []
        for time, value in zip(times, phase_values):
            if study == "steady":
                enabled = True
            elif study == "load_rejection":
                enabled = time <= event_on or time > event_off
            else:
                enabled = event_on < time <= event_off
            injected.append(value if enabled else 0.0)
        gated.append(injected)
    return gated


def main():
    parser = argparse.ArgumentParser(
        description="Plot a balanced RLC three-bus EMT standard-study monitor CSV"
    )
    parser.add_argument(
        "study",
        choices=("steady", "fault", "slg_fault", "load_rejection"),
        help="study represented by the input CSV",
    )
    parser.add_argument("csv_file", type=Path, help="monitor CSV to plot")
    parser.add_argument(
        "--output",
        type=Path,
        help="output PNG; defaults to the input CSV path with a .png suffix",
    )
    parser.add_argument("--event-on", type=float, default=0.050)
    parser.add_argument("--event-off", type=float, default=0.100)
    parser.add_argument(
        "--integration-mode",
        choices=("adaptive", "fixed"),
        default="adaptive",
    )
    args = parser.parse_args()
    if args.study != "steady" and not 0.0 <= args.event_on < args.event_off:
        raise RuntimeError("event times must satisfy 0 <= event-on < event-off")

    headers, rows, times = read_csv(args.csv_file)
    configuration = study_configuration(args.study)
    voltage = read_phase_values(headers, rows, configuration["voltage_prefix"])
    source = read_phase_values(headers, rows, "VoltageSource_source_1_i")
    internal_current = read_phase_values(
        headers, rows, configuration["current_prefix"]
    )
    injected_current = gate_injected_current(
        args.study,
        times,
        internal_current,
        args.event_on,
        args.event_off,
    )

    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    mode_label = "adaptive" if args.integration_mode == "adaptive" else "fixed step"
    monitor_step_us = 1.0e6 * (times[1] - times[0])
    figure.suptitle(
        f"{configuration['title']} — IDA {mode_label}, "
        f"{monitor_step_us:g} us monitor"
    )
    for phase, label in enumerate(PHASES):
        axes[0].plot(times, voltage[phase], label=f"v{label}")
        axes[1].plot(
            times,
            rolling_rms(times, voltage[phase]),
            label=f"v{label} RMS",
        )
        axes[2].plot(times, source[phase], label=f"source i{label}")
        axes[3].plot(
            times,
            injected_current[phase],
            label=f"injected i{label}",
        )

    for axis in axes:
        if configuration["event_label"] is not None:
            axis.axvspan(
                args.event_on,
                args.event_off,
                color=configuration["event_color"],
                alpha=0.12,
                label=configuration["event_label"],
            )
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncols=4)

    axes[0].set_ylabel(f"{configuration['voltage_label']} [V]")
    axes[1].set_ylabel("Voltage RMS [V]\n(monitor-sampled 1 cycle)")
    axes[1].ticklabel_format(axis="y", style="plain", useOffset=False)
    axes[2].set_ylabel("Source current [A]\n(positive into bus)")
    axes[3].set_ylabel(f"{configuration['current_label']} [A]")
    axes[3].set_xlabel("Time [s]")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))

    output = args.output if args.output is not None else args.csv_file.with_suffix(".png")
    figure.savefig(output, dpi=160)
    plt.close(figure)
    print(output)


if __name__ == "__main__":
    main()
