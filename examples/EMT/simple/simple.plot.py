#!/usr/bin/env python3

import csv
import sys

import matplotlib.pyplot as plt

input_file = sys.argv[1] if len(sys.argv) > 1 else "simple.csv"
output_file = sys.argv[2] if len(sys.argv) > 2 else "simple.png"

with open(input_file, newline="") as stream:
    rows = list(csv.DictReader(stream))

time = [float(row["time"]) for row in rows]
buses = sorted(
    {
        field[:-4]
        for field in rows[0].keys()
        if field.endswith(".v_a") and f"{field[:-4]}.v_b" in rows[0] and f"{field[:-4]}.v_c" in rows[0]
    }
)

fig, axes = plt.subplots(len(buses), 1, figsize=(8, 3.2 * len(buses)), sharex=True)
if len(buses) == 1:
    axes = [axes]

for axis, bus in zip(axes, buses):
    for phase in ["a", "b", "c"]:
        key = f"{bus}.v_{phase}"
        axis.plot(time, [float(row[key]) for row in rows], label=key)

    axis.set_ylabel("voltage [V]")
    axis.grid(True, alpha=0.35)
    axis.legend()

axes[-1].set_xlabel("time [s]")
plt.tight_layout()
plt.savefig(output_file, dpi=160)
