#!/usr/bin/env python3

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_float(value):
    return float(value) if value else None


def read_rows(path):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                {
                    "category": row["category"],
                    "case": row["case"],
                    "case_label": row["case_label"],
                    "tau": parse_float(row["tau"]),
                    "fmax": parse_float(row["fmax"]),
                    "N": int(row["N"]),
                    "T": parse_float(row["T"]),
                    "method": row["method"],
                    "h": parse_float(row["h"]),
                    "h_over_T": parse_float(row["h_over_T"]),
                    "h_over_tau": parse_float(row["h_over_tau"]),
                    "t": parse_float(row["t"]),
                    "input": parse_float(row["input"]),
                    "ideal_delay": parse_float(row["ideal_delay"]),
                    "output": parse_float(row["output"]),
                    "error": parse_float(row["error"]),
                }
            )
    return rows


def format_number(value):
    return f"{value:g}"


def plot_case(case_rows, output_dir, write_compat_alias=False):
    category = case_rows[0]["category"]
    case_slug = case_rows[0]["case"]
    case_label = case_rows[0]["case_label"]
    tau = case_rows[0]["tau"]
    fmax = case_rows[0]["fmax"]
    section_count = case_rows[0]["N"]
    section_time = case_rows[0]["T"]
    inv_tau = 1.0 / tau

    history = [row for row in case_rows if row["method"] == "input_history"]

    by_method = defaultdict(list)
    for row in case_rows:
        if row["method"] != "input_history":
            by_method[row["method"]].append(row)

    methods = list(by_method.keys())
    reference = by_method[methods[0]]
    input_rows = sorted(history + reference, key=lambda row: row["t"])
    ideal_rows = [row for row in reference if row["ideal_delay"] is not None]

    fig_height = 2.1 * (len(methods) + 1) + 1.0
    fig, axes = plt.subplots(len(methods) + 1, 1, sharex=True, figsize=(13, fig_height))

    fig.suptitle(
        f"{case_label}    tau={format_number(tau)} s, 1/tau={format_number(inv_tau)} Hz, "
        f"f_max={format_number(fmax)} Hz, N={section_count}, T={format_number(section_time)} s",
        fontsize=13,
    )

    top = axes[0]
    top.plot(
        [row["t"] for row in input_rows],
        [row["input"] for row in input_rows],
        label="input u(t)",
        color="black",
        linewidth=1.15,
    )
    top.plot(
        [row["t"] for row in ideal_rows],
        [row["ideal_delay"] for row in ideal_rows],
        label="ideal u(t - tau)",
        color="tab:gray",
        linestyle="--",
        linewidth=1.15,
    )
    top.axvline(0.0, color="0.6", linewidth=0.8, alpha=0.7)
    top.set_title("Input history and ideal delayed input", loc="left", fontsize=10)
    top.set_ylabel("signal")
    top.legend(loc="best")
    top.grid(True, alpha=0.25)

    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple"]
    for index, method in enumerate(methods):
        ax = axes[index + 1]
        method_rows = sorted(
            [row for row in by_method[method] if row["output"] is not None],
            key=lambda row: row["t"],
        )
        max_error = max(abs(row["error"]) for row in method_rows if row["error"] is not None)

        ax.plot(
            [row["t"] for row in ideal_rows],
            [row["ideal_delay"] for row in ideal_rows],
            label="ideal u(t - tau)",
            color="tab:gray",
            linestyle="--",
            linewidth=1.0,
        )
        ax.plot(
            [row["t"] for row in method_rows],
            [row["output"] for row in method_rows],
            label="response",
            color=colors[index % len(colors)],
            linewidth=1.0,
        )
        ax.axvline(0.0, color="0.6", linewidth=0.8, alpha=0.7)
        method_note = ""
        if "forward Euler fixed" in method:
            h = method_rows[0]["h"]
            method_note = (
                f", h={format_number(h)} s, h/T={format_number(method_rows[0]['h_over_T'])}, "
                f"h/tau={format_number(method_rows[0]['h_over_tau'])}, 1/h={format_number(1.0 / h)} Hz"
            )
        ax.set_title(f"{method}{method_note}    max |error|={max_error:.3e}", loc="left", fontsize=10)
        ax.set_ylabel("output")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.25)

    axes[-1].set_xlabel("time [s]")
    axes[-1].set_xlim(min(row["t"] for row in input_rows), max(row["t"] for row in reference))

    fig.tight_layout(rect=(0, 0, 1, 0.97))

    case_dir = output_dir / category
    case_dir.mkdir(parents=True, exist_ok=True)

    path = case_dir / f"delay_response_{case_slug}.png"
    fig.savefig(path, dpi=160)
    if write_compat_alias:
        fig.savefig(output_dir / "delay_response.png", dpi=160)
    plt.close(fig)
    return path


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("delay_response.csv")
    output_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(csv_path)
    if not rows:
        raise RuntimeError("No rows found in delay response CSV.")

    by_case = defaultdict(list)
    for row in rows:
        by_case[row["case"]].append(row)

    for index, case_rows in enumerate(by_case.values()):
        path = plot_case(case_rows, output_dir, write_compat_alias=(index == 0))
        print(path)


if __name__ == "__main__":
    main()
