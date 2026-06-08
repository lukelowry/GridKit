"""Table/report helpers for validation outputs."""

from __future__ import annotations

import json
from pathlib import Path

from cases import Case, load_study, resolve_model_path


DASH = "-"
NON_MODEL_CLASSES = {"BusFault"}

DEFAULT_TABLE_LABELS = {
    "newengland": "IEEE 39-bus",
    "hawaii": "Hawaii 37-bus",
    "illinois": "ACTIVSg200",
    "wecc": "WECC 240-bus",
    "texas": "ACTIVSg2000",
}

HEADER = r"""\begin{table*}[!t]
\centering
    \caption{Validation error of adaptive stepping against reference solution}
    \label{tab:verification-errors}
    \begin{tabular}{l rr rr rr}
    \toprule
     & & & \multicolumn{2}{c}{RMSE} & \multicolumn{2}{c}{Max error} \\
    \cmidrule(lr){4-5} \cmidrule(lr){6-7}
    Case & Buses & Models & $\omega$ & $|V|$ & $\omega$ & $|V|$ \\
    \midrule"""

FOOTER = r"""    \bottomrule
    \end{tabular}
\end{table*}"""


def _fmt(value: float | None) -> str:
    if value is None:
        return DASH
    mantissa, exponent = f"{value:.2e}".split("e")
    exp = int(exponent)
    return f"{mantissa}\\text{{e{'-' if exp < 0 else '+'}}}{abs(exp)}"


def _cell(report: dict | None, group: str, measure: str) -> str:
    if not report or not report.get(group):
        return DASH
    return _fmt(report[group].get(measure))


def _case_size(case: Case) -> tuple[int, int]:
    study = load_study(case)
    model = json.loads(resolve_model_path(case, study).read_text(encoding="utf-8"))
    buses = len(model.get("buses", []))
    models = sum(1 for dev in model.get("devices", []) if dev.get("class") not in NON_MODEL_CLASSES)
    return buses, models


def _load_error_report(output_root: Path, case: Case) -> dict | None:
    path = output_root / case.key / f"{case.key}.error.json"
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def render_error_table(cases: list[Case], output_root: Path, use_publication_labels: bool = True) -> str:
    lines = [HEADER]
    for case in cases:
        report = _load_error_report(output_root, case)
        buses, models = _case_size(case)
        label = DEFAULT_TABLE_LABELS.get(case.key, case.label) if use_publication_labels else case.label
        cells = [
            _cell(report, "omega", "rmse"),
            _cell(report, "vm", "rmse"),
            _cell(report, "omega", "max"),
            _cell(report, "vm", "max"),
        ]
        lines.append(f"    {label:<13} & {buses:>4} & {models:>5} & " + " & ".join(cells) + r" \\")
    lines.append(FOOTER)
    return "\n".join(lines) + "\n"


def write_error_table(cases: list[Case], output_root: Path, output: Path,
                      use_publication_labels: bool = True) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    text = render_error_table(cases, output_root, use_publication_labels)
    output.write_text(text, encoding="utf-8")
    print(text)
    print(f"Wrote {output}")
