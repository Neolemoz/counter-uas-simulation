"""End-to-end sensor realism propagation: fused_detections and tracks/state."""

from __future__ import annotations

from pathlib import Path

from counter_uas.sensor_realism_overlays import (
    COMBINED_OVERLAY_IDS,
    load_combined_sensor_overrides,
    load_overlay_document,
)
from counter_uas.sensor_realism_propagation import (
    baseline_profile,
    overlay_profile_from_docs,
    run_propagation,
)

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_overlay_yaml_documents_exist_and_merge() -> None:
    for overlay_id in COMBINED_OVERLAY_IDS:
        doc = load_overlay_document(overlay_id)
        assert doc.get('overlay_id') == overlay_id
    merged = load_combined_sensor_overrides()
    assert merged['radar_sim_node']['radar']['publish_every_n'] == 2
    assert merged['camera_sim_node']['camera']['publish_every_n'] == 3


def test_bringup_declares_sensor_realism_overlay_default_off() -> None:
    source = (_REPO_ROOT / 'src/counter_uas/launch/bringup.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument(\n                'enable_sensor_realism_overlay',\n                default_value='false'" in source
    assert 'config_sensor_realism_overlay.yaml' in source
    assert 'OpaqueFunction(function=launch_setup)' in source


def test_baseline_full_cadence_reaches_fused_and_tracks_state() -> None:
    counts = run_propagation(baseline_profile(), n_gt_callbacks=30, seed=11)
    assert counts.gt_callbacks == 30
    assert counts.decimated_callbacks == 0
    assert counts.radar_published == 30
    assert counts.camera_published == 30
    assert counts.fused_published >= 25
    assert counts.tracks_state_published >= 1


def test_overlay_reduces_detection_cadence_and_propagates() -> None:
    baseline = run_propagation(baseline_profile(), n_gt_callbacks=36, seed=21)
    overlay = run_propagation(overlay_profile_from_docs(), n_gt_callbacks=36, seed=21)

    assert overlay.decimated_callbacks > baseline.decimated_callbacks
    assert overlay.radar_published < baseline.radar_published
    assert overlay.camera_published < baseline.camera_published
    assert overlay.fused_published < baseline.fused_published
    assert overlay.fused_published >= 1
    assert overlay.tracks_state_published >= 1
    assert overlay.delayed_callbacks > 0


def test_overlay_range_noise_propagates_to_fused() -> None:
    baseline = run_propagation(baseline_profile(), n_gt_callbacks=36, seed=31)
    overlay = run_propagation(overlay_profile_from_docs(), n_gt_callbacks=36, seed=31)
    assert overlay.mean_fused_error_near_m >= baseline.mean_fused_error_near_m
