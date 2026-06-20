from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
CSV_PATH = HERE / "propagation.response.csv"
PNG_PATH = HERE / "propagation.response.png"


def read_columns(path: Path) -> tuple[list[float], dict[str, list[float]]]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{path} must contain a header")

        t: list[float] = []
        columns: dict[str, list[float]] = {
            name: []
            for name in reader.fieldnames
            if name.startswith("System_input_") or name.startswith("System_output_")
        }

        for row in reader:
            t.append(float(row["t"]))
            for name in columns:
                columns[name].append(float(row[name]))

    return t, columns


def main() -> None:
    t, columns = read_columns(CSV_PATH)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    for name, values in columns.items():
        if name.startswith("System_input_"):
            axes[0].plot(t, values, label=name.removeprefix("System_"))
        else:
            axes[1].plot(t, values, label=name.removeprefix("System_"))

    axes[0].set_ylabel("input")
    axes[1].set_ylabel("output")
    axes[1].set_xlabel("t [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    fig.tight_layout()
    fig.savefig(PNG_PATH, dpi=160)
    print(f"wrote {PNG_PATH}")


if __name__ == "__main__":
    main()
