"""Regression tests for paired MC G-bucket classification."""

from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_pairer():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "evaluation" / "pair_mc_seed_outcomes.py"
    assert path.is_file(), f"missing {path}"
    spec = importlib.util.spec_from_file_location("pair_mc_seed_outcomes", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def test_success_rows_with_missing_metrics_get_missing_bucket(tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001
    pairer = _load_pairer()
    baseline = tmp_path / "baseline.csv"
    candidate = tmp_path / "candidate.csv"
    out_csv = tmp_path / "paired.csv"
    fields = {
        "seed": "7",
        "success": "true",
        "miss_distance_m": "",
        "intercept_time_s": "",
        "log_path": "",
    }
    _write_csv(baseline, [fields])
    _write_csv(candidate, [fields])

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "pair_mc_seed_outcomes.py",
            str(baseline),
            str(candidate),
            "--out-csv",
            str(out_csv),
        ],
    )

    assert pairer.main() == 0
    with out_csv.open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["bucket"] == "G_metric_missing"
    assert rows[0]["miss_b"] == ""
    assert rows[0]["tint_c"] == ""
