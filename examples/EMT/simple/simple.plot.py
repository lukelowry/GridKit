#!/usr/bin/env python3

import csv
import sys

import matplotlib.pyplot as plt

input_file = sys.argv[1] if len(sys.argv) > 1 else "simple.csv"
output_file = sys.argv[2] if len(sys.argv) > 2 else "simple.png"

with open(input_file, newline="") as stream:
    rows = list(csv.DictReader(stream))

time = [float(row["time"]) for row in rows]
fields = rows[0].keys()
lines = sorted(
    field[:-4]
    for field in fields
    if field.endswith(".i_a") and f"{field[:-4]}.i_b" in fields and f"{field[:-4]}.i_c" in fields
)

if not lines:
    raise RuntimeError("No three-phase line current columns found")

line = lines[0]
bus = "bus2"

fig, axes = plt.subplots(2, 1, figsize=(8, 6.4), sharex=True)

for phase in ["a", "b", "c"]:
    key = f"{line}.i_{phase}"
    axes[0].plot(time, [float(row[key]) for row in rows], label=key)

for phase in ["a", "b", "c"]:
    key = f"{bus}.v_{phase}"
    axes[1].plot(time, [float(row[key]) for row in rows], label=key)

axes[0].set_ylabel("current [A]")
axes[1].set_ylabel("voltage [V]")

for axis in axes:
    axis.grid(True, alpha=0.35)
    axis.legend()

axes[-1].set_xlabel("time [s]")
plt.tight_layout()
plt.savefig(output_file, dpi=160)
