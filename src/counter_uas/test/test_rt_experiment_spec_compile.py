"""Tests for PLAT-RT-F5 rt_experiment_spec_compile.py."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.rt import rt_experiment_spec_compile as compile_mod

_REPO = Path(__file__).resolve().parents[3]
_EXAMPLES = _REPO / "fixtures" / "rt_experiments" / "f5_spec_examples"


@pytest.mark.parametrize(
    ("name", "run_count"),
    [
        ("terrain_comparison.json", 2),
        ("sensor_range_comparison.json", 2),
        ("tactical_mode_comparison.json", 3),
        ("repeatability_sweep.json", 3),
        ("parameter_matrix.json", 4),
    ],
)
def test_compile_f5_examples(name: str, run_count: int) -> None:
    spec = compile_mod._load_spec(_EXAMPLES / name)
    batch = compile_mod.compile_experiment_spec(spec)
    assert batch["schema"] == "rt_experiment_batch_v1"
    assert len(batch["runs"]) == run_count
    assert all(len(r.get("spec_fingerprint", "")) == 16 for r in batch["runs"])


def test_fingerprint_stable() -> None:
    spec = compile_mod._load_spec(_EXAMPLES / "parameter_matrix.json")
    assert compile_mod.compute_spec_fingerprint(spec) == compile_mod.compute_spec_fingerprint(spec)


def test_fingerprint_matches_typescript() -> None:
    """Python fingerprint must match vitest golden (PLAT-RT-F5 TS/Python parity)."""
    spec = compile_mod._load_spec(_EXAMPLES / "parameter_matrix.json")
    py_fp = compile_mod.compute_spec_fingerprint(spec)
    golden_path = _REPO / "fixtures" / "rt_experiments" / "f5_compile_goldens" / "parameter_matrix.fingerprint.txt"
    assert golden_path.read_text(encoding="utf-8").strip() == py_fp


def test_rejects_unknown_template() -> None:
    spec = compile_mod._load_spec(_EXAMPLES / "sensor_range_comparison.json")
    spec["spec_entries"][0]["template_id"] = "not_a_real_template"
    with pytest.raises(ValueError, match="unknown template_id"):
        compile_mod.compile_experiment_spec(spec)


def test_cli_writes_yaml(tmp_path: Path) -> None:
    out = tmp_path / "batch.yaml"
    rc = compile_mod.main(
        ["--spec", str(_EXAMPLES / "parameter_matrix.json"), "--out", str(out), "--format", "yaml"]
    )
    assert rc == 0
    text = out.read_text(encoding="utf-8")
    assert "rt_experiment_batch_v1" in text
    assert "runs:" in text


def test_ts_python_run_ids_match_parameter_matrix() -> None:
    spec = compile_mod._load_spec(_EXAMPLES / "parameter_matrix.json")
    py_batch = compile_mod.compile_experiment_spec(spec)
    py_ids = sorted(r["run_id"] for r in py_batch["runs"])
    golden_path = _REPO / "fixtures" / "rt_experiments" / "f5_compile_goldens" / "parameter_matrix.batch.yaml"
    import yaml

    golden = yaml.safe_load(golden_path.read_text(encoding="utf-8"))
    golden_ids = sorted(r["run_id"] for r in golden["runs"])
    assert py_ids == golden_ids
