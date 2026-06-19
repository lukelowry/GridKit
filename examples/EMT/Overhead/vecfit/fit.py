from __future__ import annotations

import csv
import math
import re
import shutil
import subprocess
from pathlib import Path

from mps import HMIN_CSV, hmin_value, tau_min_by_mode, write_mps_inputs_from_rows


HERE = Path(__file__).resolve().parent
EXAMPLE_DIR = HERE.parent
INPUT_CSV = EXAMPLE_DIR / "overhead.response.csv"
OUTPUT_DIR = EXAMPLE_DIR / "output"
YC_CSV = OUTPUT_DIR / "yc.csv"
FIN_CSV = OUTPUT_DIR / "Fin.csv"
FOUT_CSV = OUTPUT_DIR / "Fout.csv"
YC_MODEL_JSON = OUTPUT_DIR / "yc.model.json"
HMIN_MODEL_JSON = OUTPUT_DIR / "hmin.model.json"
FIN_MODEL_JSON = OUTPUT_DIR / "Fin.model.json"
FOUT_MODEL_JSON = OUTPUT_DIR / "Fout.model.json"
YC_POLES = 10
HMIN_POLES = 20
FIN_POLES = 20
FOUT_POLES = 20

MONITOR_MATRIX_COLUMN = re.compile(r"^Overhead_(\w+)_(real|imag)_(\d+)_(\d+)$")


def coordinate_column(fieldnames: list[str]) -> str:
    if "omega" in fieldnames:
        return "omega"
    if "t" in fieldnames:
        return "t"
    raise ValueError(f"{INPUT_CSV} must contain an omega column")


def read_monitor_csv() -> tuple[list[str], str, list[dict[str, str]]]:
    with INPUT_CSV.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{INPUT_CSV} must contain a header")

        fieldnames = reader.fieldnames
        omega_column = coordinate_column(fieldnames)
        return fieldnames, omega_column, list(reader)


def complex_matrix_columns(
    fieldnames: list[str],
    variable: str,
) -> tuple[int, int, list[tuple[int, int]]]:
    columns: dict[str, set[tuple[int, int]]] = {"real": set(), "imag": set()}

    for name in fieldnames:
        match = MONITOR_MATRIX_COLUMN.match(name)
        if match is None:
            continue
        parsed_variable, part, row, col = match.groups()
        if parsed_variable != variable:
            continue
        columns[part].add((int(row), int(col)))

    real = columns["real"]
    imag = columns["imag"]
    if not real:
        raise ValueError(f"{INPUT_CSV} contains no Overhead_{variable} monitor columns")
    if real != imag:
        missing_imag = sorted(real - imag)
        missing_real = sorted(imag - real)
        raise ValueError(
            f"{variable} real/imag columns are mismatched: "
            f"missing imag {missing_imag}, missing real {missing_real}"
        )

    rows = max(row for row, _ in real) + 1
    cols = max(col for _, col in real) + 1
    expected = {(row, col) for row in range(rows) for col in range(cols)}
    if real != expected:
        missing = sorted(expected - real)
        raise ValueError(f"{variable} matrix columns are not complete: missing {missing}")

    return rows, cols, sorted(real)


def monitor_complex(input_row: dict[str, str], variable: str, row: int, col: int) -> complex:
    return complex(
        float(input_row[f"Overhead_{variable}_real_{row}_{col}"]),
        float(input_row[f"Overhead_{variable}_imag_{row}_{col}"]),
    )


def frequency_hz(input_row: dict[str, str], omega_column: str) -> float:
    return float(input_row[omega_column]) / (2.0 * math.pi)


def write_vecfit_yc_csv(
    monitor_rows: list[dict[str, str]],
    omega_column: str,
    entries: list[tuple[int, int]],
) -> None:
    with YC_CSV.open("w", newline="") as out_stream:
        writer = csv.writer(out_stream)
        header = ["freq_Hz"]
        for row, col in entries:
            header.extend([f"re_Yc_{row}_{col}", f"im_Yc_{row}_{col}"])
        writer.writerow(header)

        for input_row in monitor_rows:
            output_row = [f"{frequency_hz(input_row, omega_column):.17e}"]
            for row, col in entries:
                output_row.append(input_row[f"Overhead_Yc_real_{row}_{col}"])
                output_row.append(input_row[f"Overhead_Yc_imag_{row}_{col}"])
            writer.writerow(output_row)


def write_vecfit_complex_matrix_csv(
    csv_path: Path,
    name: str,
    rows: int,
    cols: int,
    monitor_rows: list[dict[str, str]],
    omega_column: str,
    value,
) -> None:
    with csv_path.open("w", newline="") as out_stream:
        writer = csv.writer(out_stream)
        header = ["freq_Hz"]
        for row in range(rows):
            for col in range(cols):
                header.extend([f"re_{name}_{row}_{col}", f"im_{name}_{row}_{col}"])
        writer.writerow(header)

        for input_row in monitor_rows:
            output_row = [f"{frequency_hz(input_row, omega_column):.17e}"]
            omega = float(input_row[omega_column])
            for row in range(rows):
                for col in range(cols):
                    entry = value(input_row, omega, row, col)
                    output_row.extend([f"{entry.real:.17e}", f"{entry.imag:.17e}"])
            writer.writerow(output_row)


def write_vecfit_propagation_csvs(
    fieldnames: list[str],
    monitor_rows: list[dict[str, str]],
    omega_column: str,
    modes: list[int],
    conductor_count: int,
) -> None:
    tv_rows, tv_cols, _ = complex_matrix_columns(fieldnames, "Tv")
    ti_rows, ti_cols, _ = complex_matrix_columns(fieldnames, "Ti")
    mode_count = len(modes)

    if modes != list(range(mode_count)):
        raise ValueError(f"Propagation modes must be contiguous from zero: {modes}")
    if tv_rows != conductor_count or ti_rows != conductor_count:
        raise ValueError(
            f"Tv/Ti row dimensions must match conductor count {conductor_count}: "
            f"Tv is {tv_rows}x{tv_cols}, Ti is {ti_rows}x{ti_cols}"
        )
    if tv_cols != mode_count or ti_cols != mode_count:
        raise ValueError(
            f"Tv/Ti column dimensions must match modal count {mode_count}: "
            f"Tv is {tv_rows}x{tv_cols}, Ti is {ti_rows}x{ti_cols}"
        )

    tau_min = tau_min_by_mode(monitor_rows, modes)

    def fin_value(input_row: dict[str, str], omega: float, mode: int, conductor: int) -> complex:
        hmin = hmin_value(input_row, mode, tau_min[mode], omega)
        tv_h = monitor_complex(input_row, "Tv", conductor, mode).conjugate()
        return hmin * tv_h

    def fout_value(input_row: dict[str, str], omega: float, conductor: int, mode: int) -> complex:
        _ = omega
        return monitor_complex(input_row, "Ti", conductor, mode)

    write_vecfit_complex_matrix_csv(
        FIN_CSV,
        "Fin",
        mode_count,
        conductor_count,
        monitor_rows,
        omega_column,
        fin_value,
    )
    write_vecfit_complex_matrix_csv(
        FOUT_CSV,
        "Fout",
        conductor_count,
        mode_count,
        monitor_rows,
        omega_column,
        fout_value,
    )

    print(f"wrote {mode_count}x{conductor_count} Fin vecfit CSV: {FIN_CSV}")
    print(f"wrote {conductor_count}x{mode_count} Fout vecfit CSV: {FOUT_CSV}")


def write_vecfit_inputs() -> None:
    fieldnames, omega_column, monitor_rows = read_monitor_csv()
    yc_rows, yc_cols, yc_entries = complex_matrix_columns(fieldnames, "Yc")
    if yc_rows != yc_cols:
        raise ValueError(f"Yc must be square to determine conductor count: {yc_rows}x{yc_cols}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    write_vecfit_yc_csv(monitor_rows, omega_column, yc_entries)
    modes = write_mps_inputs_from_rows(
        fieldnames,
        omega_column,
        monitor_rows,
        INPUT_CSV,
        OUTPUT_DIR,
    )
    write_vecfit_propagation_csvs(
        fieldnames,
        monitor_rows,
        omega_column,
        modes,
        yc_rows,
    )

    print(f"wrote {yc_rows}x{yc_cols} Yc vecfit CSV: {YC_CSV}")

    fit_vecfit_model(YC_CSV, YC_MODEL_JSON, yc_rows, yc_cols, YC_POLES)
    fit_vecfit_model(
        HMIN_CSV,
        HMIN_MODEL_JSON,
        len(modes),
        1,
        HMIN_POLES,
        terms="none",
        state_space=True,
    )
    fit_vecfit_model(
        FIN_CSV,
        FIN_MODEL_JSON,
        len(modes),
        yc_rows,
        FIN_POLES,
        terms="none",
        state_space=True,
    )
    fit_vecfit_model(
        FOUT_CSV,
        FOUT_MODEL_JSON,
        yc_rows,
        len(modes),
        FOUT_POLES,
        terms="none",
        state_space=True,
    )


def fit_vecfit_model(
    csv_path: Path,
    model_path: Path,
    rows: int,
    cols: int,
    poles: int,
    terms: str | None = None,
    state_space: bool = False,
) -> None:
    vecfit = shutil.which("vecfit")
    if vecfit is None:
        cargo_vecfit = Path.home() / ".cargo" / "bin" / "vecfit"
        if cargo_vecfit.exists():
            vecfit = str(cargo_vecfit)
    if vecfit is None:
        raise RuntimeError("vecfit CLI not found on PATH")

    command = [
        vecfit,
        "fit",
        str(csv_path),
        "--matrix",
        str(rows),
        str(cols),
        "--poles",
        str(poles),
        "--weighted",
    ]
    if terms is not None:
        command.extend(["--terms", terms])
    if state_space:
        command.append("--state-space")
    command.extend(["--output", str(model_path)])

    subprocess.run(command, check=True)
    print(f"wrote {rows}x{cols} {poles}-pole vecfit model: {model_path}")


write_vecfit_inputs()
