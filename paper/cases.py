"""Validation case manifest and path helpers."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = REPO_ROOT / "build"
EXAMPLES = REPO_ROOT / "examples" / "PhasorDynamics"


@dataclass(frozen=True)
class Case:
    key: str
    label: str
    solver_json: Path
    sweep_by_default: bool


CASES = (
    Case("hawaii", "Hawaii", EXAMPLES / "Medium/Hawaii/hawaii.solver.json", True),
    Case("newengland", "New England", EXAMPLES / "Medium/NewEngland/newengland.solver.json", True),
    Case("illinois", "Illinois", EXAMPLES / "Large/Illinois/illinois.solver.json", False),
    Case("texas", "Texas", EXAMPLES / "Large/Texas/texas.solver.json", False),
    Case("wecc", "WECC", EXAMPLES / "Large/WECC/wecc.solver.json", False),
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_study(case: Case) -> dict[str, Any]:
    return load_json(case.solver_json)


def resolve_model_path(case: Case, study: dict[str, Any] | None = None) -> Path:
    study = study if study is not None else load_study(case)
    raw_path = Path(study["system_model_file"])
    return raw_path if raw_path.is_absolute() else case.solver_json.parent / raw_path


def reference_path(case: Case, study: dict[str, Any] | None = None) -> Path | None:
    study = study if study is not None else load_study(case)
    raw_ref = study.get("reference_file")
    if not raw_ref:
        return None
    path = Path(raw_ref)
    if not path.is_absolute():
        path = case.solver_json.parent / path
    return path if path.exists() else None


def by_keys(keys: list[str] | tuple[str, ...] | None) -> list[Case]:
    if not keys:
        return list(CASES)
    index = {case.key: case for case in CASES}
    missing = [key for key in keys if key not in index]
    if missing:
        known = ", ".join(sorted(index))
        raise SystemExit(f"Unknown case(s): {', '.join(missing)}. Known cases: {known}")
    return [index[key] for key in keys]
