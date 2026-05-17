#!/usr/bin/env python3
"""Fit OpenLine characteristic-admittance samples to GridKit RationalApproxData.

The fitter uses a stable, fixed-real-pole common-pole least-squares model:

    Yc(s) ~= D + sum_k R_k / (s - p_k)

with p_k < 0 in rad/s and real D, R_k matrices. That is directly consumable by
GridKit's EMT::Math::RationalApproxData without any complex-pair bookkeeping.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="Path to fit_manifest.json")
    parser.add_argument("--fit-dir", type=Path, help="Directory for generated *.fit.json sidecars")
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--pole-count", type=int, default=56)
    parser.add_argument("--min-pole-count", type=int, default=24)
    parser.add_argument("--max-pole-count", type=int, default=80)
    parser.add_argument("--target-rmse", type=float, default=1.5e-2)
    parser.add_argument("--target-max-rel", type=float, default=3.0e-2)
    parser.add_argument("--passivity-tol", type=float, default=1.0e-10)
    return parser.parse_args()


def read_samples(path: Path) -> dict[str, dict[str, Any]]:
    grouped: dict[str, dict[str, Any]] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            line_id = row["line_id"]
            entry = grouped.setdefault(
                line_id,
                {
                    "meta": {
                        "id": line_id,
                        "description": row["description"],
                        "structure": row["structure"],
                        "conductor": row["conductor"],
                        "bundle_count": int(row["bundle_count"]),
                        "bundle_spacing_m": parse_optional_float(row["bundle_spacing_m"]),
                        "shield_wire": row["shield_wire"],
                        "transposition": row["transposition"],
                        "length_m": float(row["length_m"]),
                        "conductor_temperature_c": float(row["conductor_temperature_c"]),
                        "ambient_temperature_c": float(row["ambient_temperature_c"]),
                        "earth_resistivity_ohm_m": float(row["earth_resistivity_ohm_m"]),
                        "dielectric_loss_tangent": float(row["dielectric_loss_tangent"]),
                    },
                    "rows": [],
                },
            )
            entry["rows"].append(row)
    return grouped


def parse_optional_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    return float(value)


def build_matrix_samples(entry: dict[str, Any]) -> tuple[np.ndarray, list[str], np.ndarray]:
    rows = entry["rows"]
    frequencies = sorted({float(row["frequency_hz"]) for row in rows})
    frequency_index = {frequency: index for index, frequency in enumerate(frequencies)}
    dimension = max(max(int(row["row"]), int(row["col"])) for row in rows) + 1
    nodes = [""] * dimension
    values = np.zeros((len(frequencies), dimension, dimension), dtype=np.complex128)

    for row in rows:
        k = frequency_index[float(row["frequency_hz"])]
        r = int(row["row"])
        c = int(row["col"])
        nodes[r] = row["node_row"]
        values[k, r, c] = float(row["real"]) + 1j * float(row["imag"])

    return np.asarray(frequencies, dtype=float), nodes, values


def candidate_pole_counts(args: argparse.Namespace) -> list[int]:
    counts = [args.pole_count]
    probe = args.min_pole_count
    while probe <= args.max_pole_count:
        counts.append(probe)
        probe += 8
    return sorted(set(count for count in counts if count > 0))


def fit_best_model(
    frequencies_hz: np.ndarray,
    values: np.ndarray,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    fit_idx = np.arange(0, len(frequencies_hz), 2)
    val_idx = np.arange(1, len(frequencies_hz), 2)
    if len(val_idx) == 0:
        val_idx = fit_idx

    best: tuple[np.ndarray, np.ndarray, dict[str, Any]] | None = None
    for pole_count in candidate_pole_counts(args):
        poles, coeff = fit_common_real_poles(
            frequencies_hz[fit_idx],
            values[fit_idx],
            pole_count,
        )
        fitted_validation = evaluate_model(frequencies_hz[val_idx], poles, coeff)
        metrics = validation_metrics(values[val_idx], fitted_validation, args.passivity_tol)
        metrics["pole_count"] = pole_count

        if best is None or quality_key(metrics) < quality_key(best[2]):
            best = (poles, coeff, metrics)

        if (
            metrics["relative_rmse"] <= args.target_rmse
            and metrics["max_relative_error"] <= args.target_max_rel
            and metrics["fit_positive_real_min_eig"] >= -args.passivity_tol
        ):
            break

    assert best is not None
    return best


def quality_key(metrics: dict[str, Any]) -> tuple[float, float, int]:
    passivity_penalty = max(0.0, -metrics["fit_positive_real_min_eig"])
    return (
        metrics["max_relative_error"] + 100.0 * passivity_penalty,
        metrics["relative_rmse"],
        metrics["pole_count"],
    )


def fit_common_real_poles(
    frequencies_hz: np.ndarray,
    values: np.ndarray,
    pole_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    dimension = values.shape[1]
    s = 1j * 2.0 * np.pi * frequencies_hz
    poles = -2.0 * np.pi * np.logspace(
        math.log10(float(frequencies_hz[0]) / 100.0),
        math.log10(float(frequencies_hz[-1]) * 100.0),
        pole_count,
    )
    basis = np.column_stack([np.ones_like(s)] + [1.0 / (s - pole) for pole in poles])
    scale = np.linalg.norm(np.vstack([basis.real, basis.imag]), axis=0)
    scale[scale == 0.0] = 1.0
    scaled_basis = basis / scale

    coeff = np.zeros((pole_count + 1, dimension, dimension), dtype=float)
    for row in range(dimension):
        for col in range(dimension):
            target = values[:, row, col]
            floor = 1.0e-4 * max(float(np.max(np.abs(target))), 1.0e-30)
            weights = 1.0 / np.maximum(np.abs(target), floor)
            system = np.vstack(
                [
                    (scaled_basis * weights[:, None]).real,
                    (scaled_basis * weights[:, None]).imag,
                ]
            )
            rhs = np.concatenate([(target * weights).real, (target * weights).imag])
            solved = np.linalg.lstsq(system, rhs, rcond=1.0e-12)[0] / scale
            coeff[:, row, col] = solved

    return poles, coeff


def evaluate_model(
    frequencies_hz: np.ndarray,
    poles: np.ndarray,
    coeff: np.ndarray,
) -> np.ndarray:
    s = 1j * 2.0 * np.pi * frequencies_hz
    basis = np.column_stack([np.ones_like(s)] + [1.0 / (s - pole) for pole in poles])
    flat_coeff = coeff.reshape((coeff.shape[0], -1))
    flat = basis @ flat_coeff
    dimension = coeff.shape[1]
    return flat.reshape((len(frequencies_hz), dimension, dimension))


def validation_metrics(
    reference: np.ndarray,
    fitted: np.ndarray,
    passivity_tol: float,
) -> dict[str, Any]:
    error = fitted - reference
    ref_norm = np.linalg.norm(reference.reshape((reference.shape[0], -1)), axis=1)
    err_norm = np.linalg.norm(error.reshape((error.shape[0], -1)), axis=1)
    rel = err_norm / np.maximum(ref_norm, 1.0e-30)

    source_eigs = np.asarray([min_hermitian_eig(matrix) for matrix in reference])
    fit_eigs = np.asarray([min_hermitian_eig(matrix) for matrix in fitted])
    source_sym = np.asarray([reciprocity_error(matrix) for matrix in reference])
    fit_sym = np.asarray([reciprocity_error(matrix) for matrix in fitted])

    return {
        "sample_count": int(reference.shape[0]),
        "relative_rmse": float(
            math.sqrt(float(np.sum(np.abs(error) ** 2)) / float(np.sum(np.abs(reference) ** 2)))
        ),
        "max_relative_error": float(np.max(rel)),
        "p95_relative_error": float(np.percentile(rel, 95.0)),
        "max_abs_error_siemens": float(np.max(np.abs(error))),
        "source_positive_real_min_eig": float(np.min(source_eigs)),
        "fit_positive_real_min_eig": float(np.min(fit_eigs)),
        "fit_positive_real_violation_count": int(np.sum(fit_eigs < -passivity_tol)),
        "source_reciprocity_max_rel": float(np.max(source_sym)),
        "fit_reciprocity_max_rel": float(np.max(fit_sym)),
    }


def min_hermitian_eig(matrix: np.ndarray) -> float:
    hermitian = 0.5 * (matrix + matrix.conj().T)
    return float(np.linalg.eigvalsh(hermitian).min())


def reciprocity_error(matrix: np.ndarray) -> float:
    return float(
        np.linalg.norm(matrix - matrix.T)
        / max(float(np.linalg.norm(matrix)), 1.0e-30)
    )


def rational_data(poles: np.ndarray, coeff: np.ndarray) -> dict[str, Any]:
    dimension = coeff.shape[1]
    return {
        "dimension": int(dimension),
        "d": flatten_matrix(coeff[0]),
        "e": [0.0] * (dimension * dimension),
        "real_poles": [float(value) for value in poles],
        "real_residues": [
            value
            for pole_index in range(len(poles))
            for value in flatten_matrix(coeff[pole_index + 1])
        ],
        "pair_real": [],
        "pair_imag": [],
        "pair_residue_real": [],
        "pair_residue_imag": [],
    }


def flatten_matrix(matrix: np.ndarray) -> list[float]:
    return [float(matrix[row, col]) for row in range(matrix.shape[0]) for col in range(matrix.shape[1])]


def pass_fail(metrics: dict[str, Any], args: argparse.Namespace) -> str:
    if metrics["relative_rmse"] > args.target_rmse:
        return "warn_rmse"
    if metrics["max_relative_error"] > args.target_max_rel:
        return "warn_max_error"
    if metrics["fit_positive_real_min_eig"] < -args.passivity_tol:
        return "warn_passivity"
    return "pass"


def canonical_phase_order(nodes: list[str]) -> list[str]:
    mapping = {
        "phase_a": "a",
        "phase_b": "b",
        "phase_c": "c",
        "a": "a",
        "b": "b",
        "c": "c",
    }
    phases = [mapping.get(node, node) for node in nodes]
    if phases != ["a", "b", "c"]:
        raise ValueError(f"expected OpenLine phase order phase_a,phase_b,phase_c; got {nodes}")
    return phases


def write_json(path: Path, data: dict[str, Any]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative_to_or_self(path: Path, base: Path) -> str:
    try:
        return path.relative_to(base).as_posix()
    except ValueError:
        return path.as_posix()


def main() -> None:
    args = parse_args()
    fit_dir = args.fit_dir if args.fit_dir is not None else args.output.parent / "fits"
    grouped = read_samples(args.samples)
    manifest_fits: list[dict[str, Any]] = []
    report_models: list[dict[str, Any]] = []
    fit_method = {
        "name": "fixed_real_pole_common_pole_least_squares",
        "description": (
            "Stable real poles are log-spaced over an expanded frequency band. "
            "Each Yc matrix entry is fit with real D and real residue matrices "
            "sharing the same pole set."
        ),
        "target_relative_rmse": args.target_rmse,
        "target_max_relative_error": args.target_max_rel,
        "passivity_tolerance": args.passivity_tol,
    }
    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()

    for line_id in sorted(grouped):
        entry = grouped[line_id]
        frequencies_hz, nodes, values = build_matrix_samples(entry)
        phase_order = canonical_phase_order(nodes)
        poles, coeff, validation = fit_best_model(frequencies_hz, values, args)
        fitted_all = evaluate_model(frequencies_hz, poles, coeff)
        full = validation_metrics(values, fitted_all, args.passivity_tol)

        validation["status"] = pass_fail(validation, args)
        validation["f_min_hz"] = float(frequencies_hz[0])
        validation["f_max_hz"] = float(frequencies_hz[-1])
        full["status"] = pass_fail(full, args)

        fit_name = f"{line_id}_yc"
        fit_file = fit_dir / f"{line_id}.yc.fit.json"
        sidecar = {
            "schema": "gridkit.emt.rational_fit",
            "format_version": 1,
            "fit_name": fit_name,
            "quantity": "characteristic_admittance",
            "domain": "phase",
            "realization": "pole_residue_real_sections",
            "matrix_layout": "row_major",
            "phase_order": phase_order,
            "transfer_function": {
                "variable": "s",
                "s_units": "rad/s",
                "form": "D + sE + sum(R/(s-p))",
            },
            "units": {
                "transfer": "S",
                "d": "S",
                "e": "S*s",
                "poles": "rad/s",
                "residues": "S/s",
            },
            "source": {
                "tool": "openline-compute gridkit-yc-samples",
                "convention": "Yc = Z^-1 * sqrt(Z * Y)",
                "source_phase_order": nodes,
            },
            "rational_approx": rational_data(poles, coeff),
            "validation": {
                "frequency_hz": {
                    "min": float(frequencies_hz[0]),
                    "max": float(frequencies_hz[-1]),
                    "samples": int(len(frequencies_hz)),
                },
                "holdout": validation,
                "all_samples": full,
            },
            "metadata": {
                "id": line_id,
                "generated_utc": now,
                "description": entry["meta"]["description"],
                "line": {
                    "length_m": entry["meta"]["length_m"],
                    "structure": entry["meta"]["structure"],
                    "conductor": entry["meta"]["conductor"],
                    "bundle_count": entry["meta"]["bundle_count"],
                    "bundle_spacing_m": entry["meta"]["bundle_spacing_m"],
                    "shield_wire": entry["meta"]["shield_wire"],
                    "transposition": entry["meta"]["transposition"],
                },
                "operating_point": {
                    "conductor_temperature_c": entry["meta"]["conductor_temperature_c"],
                    "ambient_temperature_c": entry["meta"]["ambient_temperature_c"],
                    "earth_resistivity_ohm_m": entry["meta"]["earth_resistivity_ohm_m"],
                    "dielectric_loss_tangent": entry["meta"]["dielectric_loss_tangent"],
                },
                "frequency_sweep": {
                    "f_min_hz": float(frequencies_hz[0]),
                    "f_max_hz": float(frequencies_hz[-1]),
                    "sample_count": int(len(frequencies_hz)),
                    "fit_samples": int((len(frequencies_hz) + 1) // 2),
                    "validation_samples": int(len(frequencies_hz) // 2),
                },
                "fit_method": fit_method,
            },
        }
        sha256 = write_json(fit_file, sidecar)
        manifest_fits.append(
            {
                "id": fit_name,
                "file": relative_to_or_self(fit_file, args.output.parent),
                "sha256": sha256,
                "quantity": "characteristic_admittance",
                "status": validation["status"],
                "relative_rmse": validation["relative_rmse"],
                "max_relative_error": validation["max_relative_error"],
            }
        )
        report_models.append(
            {
                "id": line_id,
                "fit_name": fit_name,
                "file": relative_to_or_self(fit_file, args.output.parent),
                "sha256": sha256,
                "pole_count": validation["pole_count"],
                "status": validation["status"],
                "holdout": validation,
                "all_samples": full,
            }
        )

        print(
            f"{line_id}: {validation['status']} poles={validation['pole_count']} "
            f"rmse={validation['relative_rmse']:.3e} "
            f"max={validation['max_relative_error']:.3e} "
            f"minPR={validation['fit_positive_real_min_eig']:.3e}"
        )

    manifest = {
        "schema": "gridkit.emt.fit_manifest",
        "format_version": 1,
        "created_utc": now,
        "sample_source": relative_to_or_self(args.samples, args.output.parent),
        "fit_method": fit_method,
        "fits": manifest_fits,
    }
    report = {
        "schema": "gridkit.emt.transmission_line_yc_validation_report",
        "format_version": 1,
        "created_utc": now,
        "model_count": len(report_models),
        "pass_count": sum(1 for model in report_models if model["status"] == "pass"),
        "models": report_models,
    }

    write_json(args.output, manifest)
    write_json(args.report, report)


if __name__ == "__main__":
    main()
