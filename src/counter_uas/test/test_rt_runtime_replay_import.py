from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_CAPTURE_GOLDEN = _REPO / "fixtures" / "rt_sandbox" / "runtime_run_capture_golden_v1.json"
_MAPPING_GOLDEN = _REPO / "fixtures" / "rt_sandbox" / "runtime_run_replay_mapping_golden_v1.json"


def _load_module(name: str, rel_path: str):  # noqa: ANN201
    path = _REPO / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_golden_capture_dry_run_matches_fixture() -> None:
    bundle_mod = _load_module("rt_runtime_replay_bundle", "scripts/evaluation/rt_runtime_replay_bundle.py")
    golden = json.loads(_MAPPING_GOLDEN.read_text(encoding="utf-8"))
    artifact = bundle_mod.load_runtime_run(_CAPTURE_GOLDEN)
    preview = bundle_mod.build_dry_run_preview(artifact, source_path=str(_CAPTURE_GOLDEN.relative_to(_REPO)))

    assert preview["mapping_ok"] is True
    assert preview["tracks_count"] == golden["dry_run_preview"]["tracks_count"]
    assert preview["entities_count"] == golden["dry_run_preview"]["entities_count"]
    assert preview["assignment_summary"] == golden["dry_run_preview"]["assignment_summary"]


def test_golden_bundle_build_lints() -> None:
    bundle_mod = _load_module("rt_runtime_replay_bundle", "scripts/evaluation/rt_runtime_replay_bundle.py")
    golden = json.loads(_MAPPING_GOLDEN.read_text(encoding="utf-8"))
    artifact = bundle_mod.load_runtime_run(_CAPTURE_GOLDEN)
    built = bundle_mod.build_replay_sa_bundle_from_runtime_run(
        artifact,
        source_path=str(_CAPTURE_GOLDEN.relative_to(_REPO)),
    )
    lint = bundle_mod.lint_bundle_candidate(built)

    assert built["artifact_type"] == "replay_sa_bundle"
    assert built["bundle_schema_version"] == "replay_sa_bundle_v1"
    assert built["clock"]["domain"] == "runtime_capture_frame_index"
    assert len(built["tracks"]) == golden["bundle_summary"]["tracks_count"]
    assert len(built["entities_static"]) == golden["bundle_summary"]["entities_static_count"]
    assert len(built["panels"]["telemetry_series"]) >= 1
    assert lint["ok"] is True
    assert built["source_artifacts"]["runtime_capture_schema"] == "rt_runtime_run_capture_v1"


def test_dry_run_cli_json_exit_zero() -> None:
    import subprocess

    proc = subprocess.run(
        [
            sys.executable,
            str(_REPO / "scripts/rt/rt_runtime_replay_import_dry_run.py"),
            str(_CAPTURE_GOLDEN),
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["schema"] == "rt_runtime_replay_import_dry_run_v1"
    assert payload["tracks_count"] >= 1
