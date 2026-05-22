from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_VALLEY_VARIANT = _REPO_ROOT / "fixtures" / "scenarios" / "valley_ingress_radar_shifted_north"


def _load_module(name: str, rel_path: str):  # noqa: ANN201
    path = _REPO_ROOT / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_authoring_manifest_present_on_variant() -> None:
    manifest_path = _VALLEY_VARIANT / "authoring_manifest.json"
    assert manifest_path.is_file()
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert data["artifact_type"] == "scenario_authoring_manifest_v1"
    assert data["promotion_status"] == "promoted"
    assert data["parent_pack_id"] == "valley_ingress"


def test_lint_authoring_manifest_ok() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    result = authoring.lint_authoring_manifest(_VALLEY_VARIANT)
    assert result["ok"], result


def test_pack_fingerprint_stable() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    a = authoring.compute_pack_fingerprint(_VALLEY_VARIANT)
    b = authoring.compute_pack_fingerprint(_VALLEY_VARIANT)
    assert a == b
    assert a.startswith("sha256:")


def test_stale_detection_after_touch() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    manifest = authoring.load_authoring_manifest(_VALLEY_VARIANT)
    assert manifest is not None
    assert not authoring.is_validation_stale(manifest, _VALLEY_VARIANT)


def test_promote_dry_run() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    result = authoring.promote_scenario_pack(
        _VALLEY_VARIANT,
        target_status="promoted",
        dry_run=True,
    )
    assert result["ok"]


def test_retired_statuses_in_enum() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    assert "deprecated" in authoring.PROMOTION_STATUSES
    assert "archived" in authoring.PROMOTION_STATUSES


def test_repro_check_valley_variant() -> None:
    authoring = _load_module("replay_sa_authoring", "scripts/evaluation/replay_sa_authoring.py")
    result = authoring.repro_check(_VALLEY_VARIANT)
    assert result["ok"], result


def test_integrity_audit_runs() -> None:
    integrity = _load_module(
        "replay_sa_authoring_integrity",
        "scripts/evaluation/replay_sa_authoring_integrity.py",
    )
    result = integrity.run_integrity_audit(strict=False)
    assert "pack_count" in result
    assert result["artifact_type"] == "scenario_authoring_integrity_report_v1"


def test_lineage_continuity_report() -> None:
    integrity = _load_module(
        "replay_sa_authoring_integrity",
        "scripts/evaluation/replay_sa_authoring_integrity.py",
    )
    report = integrity.lineage_continuity_report("valley_ingress_radar_shifted_north")
    assert report["depth"] >= 2
    assert report["ancestor_chain"][0]["pack_id"] == "valley_ingress_radar_shifted_north"


def test_catalog_has_full_manifest_coverage() -> None:
    integrity = _load_module(
        "replay_sa_authoring_integrity",
        "scripts/evaluation/replay_sa_authoring_integrity.py",
    )
    result = integrity.run_integrity_audit(strict=False)
    assert result["manifest_count"] == result["pack_count"]
    assert result["pack_count"] >= 11
