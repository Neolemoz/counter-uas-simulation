"""Contract tests for statistical validation MC profile CSV."""

from __future__ import annotations

import csv
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PROFILES = (
    _REPO_ROOT
    / 'scripts'
    / 'evaluation'
    / 'fixtures'
    / 'statistical_validation_profiles.csv'
)


def _load_rows() -> list[dict[str, str]]:
    with _PROFILES.open(encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def test_statistical_validation_profile_ids_are_stable() -> None:
    rows = _load_rows()
    labels = [str(row['label']).strip() for row in rows]
    assert labels == [
        'predictive_baseline',
        'predictive_intercept',
        'hysteresis_off',
        'hysteresis_on',
        'multi_defender',
        'aero_realism_on',
        'sensor_realism_on',
    ]


def test_statistical_validation_profiles_use_matched_seeds_and_explicit_arms() -> None:
    rows = _load_rows()
    assert all(str(row['n']).strip() == '2' for row in rows)
    assert all(str(row['seed_base']).strip() == '6201' for row in rows)

    by_label = {str(row['label']).strip(): row for row in rows}
    baseline = by_label['predictive_baseline']
    intercept = by_label['predictive_intercept']
    assert 'eng_rollout_feasibility_gate:=false' in baseline['launch_args']
    assert 'eng_rollout_feasibility_gate:=true' in intercept['launch_args']
    assert 'intercept_heatmap_prob_use_kinematic_rollout:=true' in intercept['launch_args']

    assert 'intercept_mc_hysteresis_enabled:=false' in by_label['hysteresis_off']['launch_args']
    assert 'intercept_mc_hysteresis_enabled:=true' in by_label['hysteresis_on']['launch_args']

    assert by_label['multi_defender']['scenario'] == 'multi'
    assert 'eng_rollout_feasibility_gate:=true' in by_label['multi_defender']['launch_args']

    aero = by_label['aero_realism_on']
    assert 'interceptor_max_turn_rate_rad_s:=0.2792526803190757' in aero['launch_args']
    assert 'interceptor_max_accel_m_s2:=30.0' in aero['launch_args']

    sensor = by_label['sensor_realism_on']
    assert sensor['scenario'] == 'bringup'
    assert 'enable_sensor_realism_overlay:=true' in sensor['launch_args']
    assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in sensor['launch_args']
    assert 'target_start_x_m:=-1500.0' in sensor['launch_args']
    assert 'target_start_z_m:=300.0' in sensor['launch_args']
