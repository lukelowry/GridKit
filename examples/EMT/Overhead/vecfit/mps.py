from __future__ import annotations

import csv
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
EXAMPLE_DIR = HERE.parent
INPUT_CSV = EXAMPLE_DIR / "overhead.response.csv"
OUTPUT_DIR = EXAMPLE_DIR / "output"
HMIN_CSV = OUTPUT_DIR / "hmin.csv"
DELAY_JSON = OUTPUT_DIR / "delay.json"


def coordinate_column(fieldnames: list[str], input_csv: Path) -> str:
    if "omega" in fieldnames:
        return "omega"
    if "t" in fieldnames:
        return "t"
    raise ValueError(f"{input_csv} must contain an omega column")


def read_monitor_csv(input_csv: Path = INPUT_CSV) -> tuple[list[str], str, list[dict[str, str]]]:
    with input_csv.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{input_csv} must contain a header")

        fieldnames = reader.fieldnames
        omega_column = coordinate_column(fieldnames, input_csv)
        return fieldnames, omega_column, list(reader)


def h_modes(fieldnames: list[str], input_csv: Path) -> list[int]:
    real_prefix = "Overhead_H_real_"
    imag_prefix = "Overhead_H_imag_"
    tau_prefix = "Overhead_Tau_"

    real = {
        int(name.removeprefix(real_prefix))
        for name in fieldnames
        if name.startswith(real_prefix)
    }
    imag = {
        int(name.removeprefix(imag_prefix))
        for name in fieldnames
        if name.startswith(imag_prefix)
    }
    tau = {
        int(name.removeprefix(tau_prefix))
        for name in fieldnames
        if name.startswith(tau_prefix)
    }

    if not real:
        raise ValueError(f"{input_csv} contains no Overhead_H monitor columns")
    if real != imag or real != tau:
        raise ValueError(
            f"H real/imag/Tau columns are mismatched: "
            f"missing imag {sorted(real - imag)}, missing real {sorted(imag - real)}, "
            f"missing tau {sorted(real - tau)}"
        )

    return sorted(real)


def frequency_hz(input_row: dict[str, str], omega_column: str) -> float:
    return float(input_row[omega_column]) / (2.0 * math.pi)


def hmin_value(input_row: dict[str, str], mode: int, tau_min: float, omega: float) -> complex:
    real = float(input_row[f"Overhead_H_real_{mode}"])
    imag = float(input_row[f"Overhead_H_imag_{mode}"])
    angle = omega * tau_min
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return complex(
        real * cos_angle - imag * sin_angle,
        real * sin_angle + imag * cos_angle,
    )


def tau_min_by_mode(
    monitor_rows: list[dict[str, str]],
    modes: list[int],
) -> dict[int, float]:
    return {
        mode: min(float(row[f"Overhead_Tau_{mode}"]) for row in monitor_rows)
        for mode in modes
    }


def write_hmin_csv(
    monitor_rows: list[dict[str, str]],
    omega_column: str,
    modes: list[int],
    hmin_csv: Path,
) -> list[float]:
    tau_min = tau_min_by_mode(monitor_rows, modes)

    with hmin_csv.open("w", newline="") as out_stream:
        writer = csv.writer(out_stream)
        header = ["freq_Hz"]
        for mode in modes:
            header.extend([f"re_Hmin_{mode}_0", f"im_Hmin_{mode}_0"])
        writer.writerow(header)

        for input_row in monitor_rows:
            omega = float(input_row[omega_column])
            output_row = [f"{frequency_hz(input_row, omega_column):.17e}"]
            for mode in modes:
                value = hmin_value(input_row, mode, tau_min[mode], omega)
                output_row.extend([f"{value.real:.17e}", f"{value.imag:.17e}"])
            writer.writerow(output_row)

    return [tau_min[mode] for mode in modes]


def write_delay_json(tau_min: list[float], delay_json: Path) -> None:
    delay_json.write_text(json.dumps(tau_min, indent=2) + "\n")


def write_mps_inputs_from_rows(
    fieldnames: list[str],
    omega_column: str,
    monitor_rows: list[dict[str, str]],
    input_csv: Path = INPUT_CSV,
    output_dir: Path = OUTPUT_DIR,
) -> list[int]:
    modes = h_modes(fieldnames, input_csv)

    output_dir.mkdir(parents=True, exist_ok=True)
    hmin_csv = output_dir / "hmin.csv"
    delay_json = output_dir / "delay.json"

    tau_min = write_hmin_csv(monitor_rows, omega_column, modes, hmin_csv)
    write_delay_json(tau_min, delay_json)

    print(f"wrote {len(modes)}x1 Hmin vecfit CSV: {hmin_csv}")
    print(f"wrote modal tau_min delays: {delay_json}")

    return modes


def write_mps_inputs(input_csv: Path = INPUT_CSV, output_dir: Path = OUTPUT_DIR) -> list[int]:
    fieldnames, omega_column, monitor_rows = read_monitor_csv(input_csv)
    return write_mps_inputs_from_rows(
        fieldnames,
        omega_column,
        monitor_rows,
        input_csv,
        output_dir,
    )


if __name__ == "__main__":
    write_mps_inputs()
