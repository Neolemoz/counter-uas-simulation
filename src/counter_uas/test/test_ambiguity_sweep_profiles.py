from __future__ import annotations

import csv
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_wave1_ambiguity_sweep_profiles_are_stable_and_explicit() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'baseline',
        'near_threshold_ghost_only',
        'staggered_fragmentation_only',
        'wave1_combined',
        'wave1_high_pressure',
    ]

    baseline = rows[0]
    assert 'ghost_placement_mode:=' not in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']

    ghost_only = rows[1]
    assert 'ghost_placement_mode:=near_threshold' in ghost_only['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in ghost_only['launch_args']

    frag_only = rows[2]
    assert 'fragmentation_staggered_enabled:=true' in frag_only['launch_args']
    assert 'ghost_placement_mode:=' not in frag_only['launch_args']

    for row in rows[1:]:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']


def test_wave2_ambiguity_sweep_profiles_are_stable_explicit_and_default_safe() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave2.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'baseline',
        'sensor_path_propagation_only',
        'fragmentation_lifecycle_pressure',
        'persistent_near_threshold_ghost_pressure',
        'wave2_combined',
        'wave2_high_pressure',
    ]

    baseline = rows[0]
    assert 'sensor_input_topic:=/drone/position_noisy' not in baseline['launch_args']
    assert 'ghost_persistence_ticks:=' not in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']

    propagation_only = rows[1]
    assert 'sensor_input_topic:=/drone/position_noisy' in propagation_only['launch_args']
    assert 'ghost_persistence_ticks:=' not in propagation_only['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in propagation_only['launch_args']

    frag_pressure = rows[2]
    assert 'sensor_input_topic:=/drone/position_noisy' in frag_pressure['launch_args']
    assert 'fragmentation_staggered_enabled:=true' in frag_pressure['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=4' in frag_pressure['launch_args']
    assert 'ghost_persistence_ticks:=' not in frag_pressure['launch_args']

    ghost_pressure = rows[3]
    assert 'sensor_input_topic:=/drone/position_noisy' in ghost_pressure['launch_args']
    assert 'ghost_placement_mode:=near_threshold' in ghost_pressure['launch_args']
    assert 'ghost_persistence_ticks:=4' in ghost_pressure['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in ghost_pressure['launch_args']

    for row in rows[1:]:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']
