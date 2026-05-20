from __future__ import annotations

import importlib.util
import json
import sys
import zipfile
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIXTURE_NARRATIVE = Path(__file__).resolve().parent / 'fixtures' / 'replay_narrative_minimal.json'
_SCENARIO_PACK = _REPO_ROOT / 'fixtures' / 'scenarios' / 'ridge_defense'

_SYNTHETIC_LOG = """
=== run_id: replay_obs_unit ===
[interception_logic_node-1] [P_HEATMAP] pos=(-1200.000, 50.000, 300.000)
[interception_logic_node-1] interceptor_pos=(-100.000, 10.000, 50.000) target_pos=(-1150.000, 45.000, 295.000)
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=12.500 m | t_go=2.5 s | vel=18.0 m/s | mode=predict
[tracking_node-1] Candidate detected: track_id=7
[noisy_measurement_node-1] [REALISM_EVENT] fragmented_gap_start fragmentation_index=1 gap_ticks=3
[interception_logic_node-1] interceptor_pos=(-80.000, 12.000, 48.000) target_pos=(-1100.000, 40.000, 290.000)
""".strip()


def _load_module(name: str, rel_path: str):  # noqa: ANN201
    path = _REPO_ROOT / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _write_log_and_meta(tmp_path: Path) -> tuple[Path, Path]:
    log = tmp_path / 'replay_obs_unit.log'
    meta = tmp_path / 'replay_obs_unit.meta.json'
    log.write_text(_SYNTHETIC_LOG, encoding='utf-8')
    meta.write_text(
        json.dumps(
            {
                'run_id': 'replay_obs_unit',
                'cmd': [
                    'ros2',
                    'launch',
                    'counter_uas',
                    'bringup.launch.py',
                    'target_start_x_m:=-1500.0',
                    'target_start_y_m:=0.0',
                    'target_start_z_m:=300.0',
                    'noise_seed:=101',
                ],
            }
        ),
        encoding='utf-8',
    )
    return log, meta


def test_build_bundle_contract(tmp_path: Path) -> None:
    sa = _load_module('replay_sa_bundle', 'scripts/evaluation/replay_sa_bundle.py')
    obs = _load_module('replay_observability', 'scripts/evaluation/replay_observability.py')
    viz = _load_module('replay_static_visualization', 'scripts/evaluation/replay_static_visualization.py')

    log, meta = _write_log_and_meta(tmp_path)
    narrative = json.loads(_FIXTURE_NARRATIVE.read_text(encoding='utf-8'))
    narrative['lineage']['log_path'] = str(log)
    narrative['lineage']['meta_path'] = str(meta)

    single = obs.build_single_run_report(log, meta_path=meta)
    manifest = viz.build_visualization_manifest(narrative, observability=single)

    bundle = sa.build_replay_sa_bundle(
        narrative,
        observability=single,
        viz_manifest=manifest,
        scenario_pack_path=_SCENARIO_PACK,
    )
    lint = sa.lint_replay_sa_bundle(bundle)

    assert bundle['artifact_type'] == 'replay_sa_bundle'
    assert bundle['source_artifacts'].get('scenario_pack') == 'fixtures/scenarios/ridge_defense'
    assert bundle['bundle_schema_version'] == 'replay_sa_bundle_v1'
    assert bundle['mode'] == 'replay_static'
    assert lint['ok'] is True
    assert bundle['tracks']
    assert bundle['entities_static']
    assert bundle['zones']
    assert bundle['overlays']
    assert bundle['los_segments']
    assert all(seg.get('caveat') for seg in bundle['los_segments'])
    assert bundle['scenario'].get('terrain_model')
    assert bundle['narrative']['annotations']
    glance = bundle['comprehension']['at_a_glance']
    assert isinstance(glance, dict)
    assert isinstance(glance['cards'], list)
    assert isinstance(glance['summary'], dict)
    assert glance['cards']


def test_pack_bundle_is_deterministic(tmp_path: Path) -> None:
    sa = _load_module('replay_sa_bundle', 'scripts/evaluation/replay_sa_bundle.py')
    obs = _load_module('replay_observability', 'scripts/evaluation/replay_observability.py')

    log, meta = _write_log_and_meta(tmp_path)
    narrative = json.loads(_FIXTURE_NARRATIVE.read_text(encoding='utf-8'))
    narrative['lineage']['log_path'] = str(log)
    narrative['lineage']['meta_path'] = str(meta)
    narr_path = tmp_path / 'narrative.json'
    narr_path.write_text(json.dumps(narrative), encoding='utf-8')
    single = obs.build_single_run_report(log, meta_path=meta)
    obs_path = tmp_path / 'obs.json'
    obs_path.write_text(json.dumps(single), encoding='utf-8')

    out_a = tmp_path / 'bundle_a'
    out_b = tmp_path / 'bundle_b'
    sa.pack_bundle(
        narrative_json=narr_path,
        observability_json=obs_path,
        scenario_pack_dir=_SCENARIO_PACK,
        out_dir=out_a,
    )
    sa.pack_bundle(
        narrative_json=narr_path,
        observability_json=obs_path,
        scenario_pack_dir=_SCENARIO_PACK,
        out_dir=out_b,
    )

    first = json.loads((out_a / 'index.json').read_text(encoding='utf-8'))
    second = json.loads((out_b / 'index.json').read_text(encoding='utf-8'))
    assert first == second


_MULTI_TRACK_LOG = """
=== run_id: replay_saturation_unit ===
[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_0 pos=(-1800.000, 40.000, 120.000)
[interception_logic_node-1] [P_HEATMAP] threat_id=threat_uav_1 pos=(-1700.000, -80.000, 110.000)
[interception_logic_node-1] [METRICS] id=interceptor_0  | dist=40.000 m | t_go=8.0 s | vel=16.0 m/s | mode=predict
[interception_logic_node-1] interceptor_id=interceptor_0 interceptor_pos=(-50.000, 5.000, 40.000) target_pos=(-1750.000, 35.000, 115.000) threat_id=threat_uav_0
[interception_logic_node-1] [METRICS] id=interceptor_1  | dist=55.000 m | t_go=9.5 s | vel=15.0 m/s | mode=predict
[interception_logic_node-1] interceptor_id=interceptor_1 interceptor_pos=(80.000, -20.000, 42.000) target_pos=(-1680.000, -75.000, 108.000) threat_id=threat_uav_1
""".strip()


def test_parse_multi_track_log(tmp_path: Path) -> None:
    sa = _load_module('replay_sa_bundle', 'scripts/evaluation/replay_sa_bundle.py')
    log = tmp_path / 'multi.log'
    log.write_text(_MULTI_TRACK_LOG, encoding='utf-8')
    tracks, telemetry = sa._parse_tracks_from_log(log)
    threat_ids = {t['track_id'] for t in tracks if t['role'] == 'threat'}
    interceptor_ids = {t['track_id'] for t in tracks if t['role'] == 'interceptor'}
    assert threat_ids == {'threat_uav_0', 'threat_uav_1'}
    assert interceptor_ids == {'interceptor_0', 'interceptor_1'}
    assert len(tracks) >= 3
    assert len(telemetry) == 2


def test_saturation_demo_bundle_multi_track() -> None:
    sa = _load_module('replay_sa_bundle', 'scripts/evaluation/replay_sa_bundle.py')
    bundle_path = _REPO_ROOT / 'fixtures' / 'sa_r0' / 'demo_saturation_ingress' / 'index.json'
    bundle = json.loads(bundle_path.read_text(encoding='utf-8'))
    lint = sa.lint_replay_sa_bundle(bundle)
    assert lint['ok'], lint
    assert len(bundle['tracks']) >= 3
    threat_ids = {t['track_id'] for t in bundle['tracks'] if t['role'] == 'threat'}
    assert 'threat_uav_0' in threat_ids
    assert 'threat_uav_1' in threat_ids


def test_export_portable_zip(tmp_path: Path) -> None:
    sa = _load_module('replay_sa_bundle', 'scripts/evaluation/replay_sa_bundle.py')
    obs = _load_module('replay_observability', 'scripts/evaluation/replay_observability.py')

    log, meta = _write_log_and_meta(tmp_path)
    narrative = json.loads(_FIXTURE_NARRATIVE.read_text(encoding='utf-8'))
    narrative['lineage']['log_path'] = str(log)
    narrative['lineage']['meta_path'] = str(meta)
    narr_path = tmp_path / 'narrative.json'
    narr_path.write_text(json.dumps(narrative), encoding='utf-8')
    single = obs.build_single_run_report(log, meta_path=meta)
    obs_path = tmp_path / 'obs.json'
    obs_path.write_text(json.dumps(single), encoding='utf-8')

    bundle_dir = tmp_path / 'bundle'
    sa.pack_bundle(
        narrative_json=narr_path,
        observability_json=obs_path,
        scenario_pack_dir=_SCENARIO_PACK,
        out_dir=bundle_dir,
    )
    out_zip = tmp_path / 'bundle.zip'
    sa.export_portable_zip(bundle_dir, out_zip)
    assert out_zip.is_file()
    with zipfile.ZipFile(out_zip) as zf:
        assert 'index.json' in zf.namelist()
