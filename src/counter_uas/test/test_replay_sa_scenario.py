from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PACK_RIDGE = _REPO_ROOT / "fixtures" / "scenarios" / "ridge_defense"
_PACK_MULTI = _REPO_ROOT / "fixtures" / "scenarios" / "multi_ridge"
_B2_PACKS = [
    "ridge_defense",
    "valley_ingress",
    "multi_ridge",
    "corridor_defense",
    "saturation_ingress",
    "urban_masking",
    "delayed_detection",
    "long_range_ingress",
]
_D1_EXPERIMENT_PACKS = [
    "valley_ingress_radar_shifted_north",
    "valley_ingress_extra_valley_sensor",
    "valley_ingress_reduced_overlap_layout",
    "valley_ingress_delayed_interceptor_base",
]


def _load_module(name: str, rel_path: str):  # noqa: ANN201
    path = _REPO_ROOT / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_lint_ridge_defense_pack_ok() -> None:
    scenario = _load_module("replay_sa_scenario", "scripts/evaluation/replay_sa_scenario.py")
    result = scenario.lint_scenario_pack(_PACK_RIDGE)
    assert result["ok"], result


def test_load_scenario_pack_merges_overlay_shape() -> None:
    scenario = _load_module("replay_sa_scenario", "scripts/evaluation/replay_sa_scenario.py")
    overlay = scenario.load_scenario_pack(_PACK_RIDGE)
    assert overlay["scenario_id"] == "ridge_defense_demo"
    assert len(overlay.get("entities_static") or []) >= 4
    assert len(overlay.get("overlays") or []) == 3
    assert overlay.get("include_fictional_terrain") is True


def test_lint_all_b2_packs_ok() -> None:
    scenario = _load_module('replay_sa_scenario', 'scripts/evaluation/replay_sa_scenario.py')
    for pack_id in _B2_PACKS:
        result = scenario.lint_scenario_pack(_REPO_ROOT / 'fixtures' / 'scenarios' / pack_id)
        assert result['ok'], (pack_id, result)


def test_c1b_metadata_on_saturation_pack() -> None:
    scenario = _load_module("replay_sa_scenario", "scripts/evaluation/replay_sa_scenario.py")
    pack = _REPO_ROOT / "fixtures/scenarios/saturation_ingress"
    result = scenario.lint_scenario_pack(pack)
    assert result["ok"], result
    meta = json.loads((pack / "metadata.json").read_text(encoding="utf-8"))
    assert meta.get("replay_tags")
    assert meta.get("ambiguity_profile", {}).get("level") == "saturation"


def test_pack_bundle_includes_c1b_scenario_fields() -> None:
    sa = _load_module("replay_sa_bundle", "scripts/evaluation/replay_sa_bundle.py")
    obs = _load_module("replay_observability", "scripts/evaluation/replay_observability.py")
    viz = _load_module("replay_static_visualization", "scripts/evaluation/replay_static_visualization.py")

    pack = _REPO_ROOT / "fixtures/scenarios/saturation_ingress"
    log = _REPO_ROOT / "fixtures/sa_r0/demo_saturation_ingress/demo.log"
    meta = _REPO_ROOT / "fixtures/sa_r0/demo_saturation_ingress/demo.meta.json"
    narrative = json.loads(
        (_REPO_ROOT / "src/counter_uas/test/fixtures/replay_narrative_saturation_ingress.json").read_text()
    )
    narrative["lineage"]["log_path"] = str(log.relative_to(_REPO_ROOT))
    narrative["lineage"]["meta_path"] = str(meta.relative_to(_REPO_ROOT))

    single = obs.build_single_run_report(log, meta_path=meta)
    manifest = viz.build_visualization_manifest(narrative, observability=single)
    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=pack,
    )
    assert bundle["scenario"].get("replay_tags")
    assert bundle["scenario"].get("ingress_archetype") == "open"
    assert bundle["comparison_hints"]["topology_key"] == "saturation_ingress"
    assert bundle["comparison_hints"].get("comparison_ready") is True


def test_lint_d1_experiment_packs_ok() -> None:
    scenario = _load_module("replay_sa_scenario", "scripts/evaluation/replay_sa_scenario.py")
    for pack_id in _D1_EXPERIMENT_PACKS:
        result = scenario.lint_scenario_pack(_REPO_ROOT / "fixtures" / "scenarios" / pack_id)
        assert result["ok"], (pack_id, result)


def test_sensor_layout_id_differs_for_experiment() -> None:
    scenario = _load_module("replay_sa_scenario", "scripts/evaluation/replay_sa_scenario.py")
    base_topo = json.loads(
        (_REPO_ROOT / "fixtures/scenarios/valley_ingress/topology.json").read_text(encoding="utf-8")
    )
    exp_topo = json.loads(
        (
            _REPO_ROOT / "fixtures/scenarios/valley_ingress_radar_shifted_north/topology.json"
        ).read_text(encoding="utf-8")
    )
    id_base = scenario.compute_sensor_layout_id(base_topo)
    id_exp = scenario.compute_sensor_layout_id(exp_topo)
    assert id_base != id_exp
    assert id_base.startswith("sha256:")
    diff = scenario.diff_topology_entities(
        base_topo["entities_static"], exp_topo["entities_static"]
    )
    assert diff["moved"]


def test_experiment_bundle_sensor_study_hints() -> None:
    sa = _load_module("replay_sa_bundle", "scripts/evaluation/replay_sa_bundle.py")
    obs = _load_module("replay_observability", "scripts/evaluation/replay_observability.py")
    viz = _load_module("replay_static_visualization", "scripts/evaluation/replay_static_visualization.py")
    pack = _REPO_ROOT / "fixtures/scenarios/valley_ingress_radar_shifted_north"
    log = _REPO_ROOT / "fixtures/sa_r0/demo_valley_ingress/demo.log"
    meta = _REPO_ROOT / "fixtures/sa_r0/demo_valley_ingress/demo.meta.json"
    narrative = json.loads(
        (_REPO_ROOT / "src/counter_uas/test/fixtures/replay_narrative_minimal.json").read_text()
    )
    narrative["lineage"]["log_path"] = str(log.relative_to(_REPO_ROOT))
    narrative["lineage"]["meta_path"] = str(meta.relative_to(_REPO_ROOT))
    single = obs.build_single_run_report(log, meta_path=meta)
    manifest = viz.build_visualization_manifest(narrative, observability=single)
    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=pack,
    )
    hints = bundle["comparison_hints"]
    assert hints.get("sensor_layout_id")
    assert hints.get("compare_mode") == "sensor_study"
    assert hints.get("baseline_topology_key") == "valley_ingress"


def test_compare_pairs_manifest_pack_ids() -> None:
    data = json.loads(
        (_REPO_ROOT / "fixtures/scenarios/compare_pairs_v1.json").read_text(encoding="utf-8")
    )
    assert data["artifact_type"] == "scenario_compare_pairs_v1"
    assert len(data["pairs"]) >= 4


def test_validate_scenario_cli() -> None:
    script = _REPO_ROOT / "scripts" / "evaluation" / "validate_scenario.py"
    proc = subprocess.run(
        [sys.executable, str(script), str(_PACK_MULTI)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr + proc.stdout


def test_pack_bundle_with_scenario_pack(tmp_path: Path) -> None:
    sa = _load_module("replay_sa_bundle", "scripts/evaluation/replay_sa_bundle.py")
    obs = _load_module("replay_observability", "scripts/evaluation/replay_observability.py")
    viz = _load_module("replay_static_visualization", "scripts/evaluation/replay_static_visualization.py")

    log = _REPO_ROOT / "fixtures" / "sa_r0" / "demo_ridge_defense" / "demo.log"
    meta = _REPO_ROOT / "fixtures" / "sa_r0" / "demo_ridge_defense" / "demo.meta.json"
    narrative_path = Path(__file__).resolve().parent / "fixtures" / "replay_narrative_minimal.json"
    narrative = json.loads(narrative_path.read_text(encoding="utf-8"))
    narrative["lineage"]["log_path"] = str(log)
    narrative["lineage"]["meta_path"] = str(meta)

    single = obs.build_single_run_report(log, meta_path=meta)
    manifest = viz.build_visualization_manifest(narrative, observability=single)

    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=_PACK_RIDGE,
    )
    assert bundle["scenario"].get("scenario_pack_id") == "ridge_defense_demo"
    assert bundle["source_artifacts"].get("scenario_pack") == "fixtures/scenarios/ridge_defense"
    log_path = bundle["lineage"]["log_path"]
    assert not str(log_path).startswith("/home/")
    assert "fixtures/sa_r0" in str(log_path)
