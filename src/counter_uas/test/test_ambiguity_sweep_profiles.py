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


def test_wave3_ambiguity_sweep_profiles_are_stable_explicit_and_default_safe() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave3.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'baseline_legacy_single',
        'bringup_topology_only',
        'cadence_aligned_coast_pressure',
        'silence_resume_oscillation_pressure',
        'combined_wave3_lifecycle_pressure',
        'wave3_high_pressure',
    ]

    baseline = rows[0]
    assert baseline['scenario'] == 'single'
    assert 'counter_uas_config:=' not in baseline['launch_args']
    assert 'sensor_input_topic:=' not in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']

    bringup_baseline = rows[1]
    assert bringup_baseline['scenario'] == 'bringup'
    assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in bringup_baseline['launch_args']
    assert 'sensor_input_topic:=/drone/position_noisy' in bringup_baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in bringup_baseline['launch_args']

    burst = rows[2]
    assert burst['scenario'] == 'bringup'
    assert 'fragmentation_staggered_enabled:=true' in burst['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=4' in burst['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=5' in burst['launch_args']

    oscillation = rows[3]
    assert oscillation['scenario'] == 'bringup'
    assert 'fragmentation_staggered_enabled:=true' in oscillation['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=4' in oscillation['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=3' in oscillation['launch_args']

    combined = rows[4]
    assert combined['scenario'] == 'bringup'
    assert 'fragmentation_staggered_enabled:=true' in combined['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=6' in combined['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=4' in combined['launch_args']

    high_pressure = rows[5]
    assert high_pressure['scenario'] == 'bringup'
    assert 'fragmentation_staggered_enabled:=true' in high_pressure['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=7' in high_pressure['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=5' in high_pressure['launch_args']

    for row in rows[1:]:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']


def test_wave5_ambiguity_sweep_profiles_are_explicit_default_off_and_threshold_sensitive() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave5.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'bringup_topology_only',
        'wave5_threshold_crossing_pressure',
        'wave5_confirmed_track_reachability',
        'wave5_recovery_cycle_pressure',
    ]

    baseline = rows[0]
    assert baseline['scenario'] == 'bringup'
    assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in baseline['launch_args']
    assert 'sensor_input_topic:=/drone/position_noisy' in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']

    pressure = rows[1]
    assert pressure['scenario'] == 'bringup'
    assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in pressure['launch_args']
    assert 'sensor_input_topic:=/drone/position_noisy' in pressure['launch_args']
    assert 'fragmentation_staggered_enabled:=true' in pressure['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=10' in pressure['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=9' in pressure['launch_args']
    assert 'ghost_persistence_ticks:=' not in pressure['launch_args']

    reachability = rows[2]
    assert reachability['scenario'] == 'bringup'
    assert 'enable_lifecycle_observer:=true' in reachability['launch_args']
    assert 'fragmentation_staggered_enabled:=true' in reachability['launch_args']
    assert 'target_start_x_m:=-1500.0' in reachability['launch_args']
    assert 'target_start_y_m:=0.0' in reachability['launch_args']
    assert 'target_start_z_m:=300.0' in reachability['launch_args']

    recovery = rows[3]
    assert recovery['scenario'] == 'bringup'
    assert 'enable_lifecycle_observer:=true' in recovery['launch_args']
    assert 'fragmentation_staggered_enabled:=true' in recovery['launch_args']
    assert 'fragmentation_stagger_cycle_ticks:=7' in recovery['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=4' in recovery['launch_args']
    assert 'target_start_x_m:=-1500.0' in recovery['launch_args']
    assert 'target_start_y_m:=0.0' in recovery['launch_args']
    assert 'target_start_z_m:=300.0' in recovery['launch_args']

    for row in rows:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']


def test_wave5_recovery_envelope_profiles_are_explicit_and_geometry_stable() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave5_recovery_envelope.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'wave5_reachability_baseline',
        'wave5_recovery_low_pressure',
        'wave5_recovery_cycle_pressure',
        'wave5_recovery_high_pressure',
    ]

    baseline = rows[0]
    assert baseline['scenario'] == 'bringup'
    assert 'enable_lifecycle_observer:=true' in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']
    assert 'target_start_x_m:=-1500.0' in baseline['launch_args']
    assert 'target_start_y_m:=0.0' in baseline['launch_args']
    assert 'target_start_z_m:=300.0' in baseline['launch_args']

    low = rows[1]
    assert 'fragmentation_stagger_cycle_ticks:=6' in low['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=3' in low['launch_args']

    mid = rows[2]
    assert 'fragmentation_stagger_cycle_ticks:=7' in mid['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=4' in mid['launch_args']

    high = rows[3]
    assert 'fragmentation_stagger_cycle_ticks:=8' in high['launch_args']
    assert 'fragmentation_stagger_gap_ticks:=5' in high['launch_args']

    for row in rows:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']
        assert 'sensor_input_topic:=/drone/position_noisy' in row['launch_args']
        assert 'enable_lifecycle_observer:=true' in row['launch_args']


def test_wave5_phase_spacing_profiles_are_explicit_and_hold_cadence_constant() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave5_phase_spacing.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        'wave5_phase_reference_baseline',
        'wave5_phase_spacing_p0',
        'wave5_phase_spacing_p1',
        'wave5_phase_spacing_p2',
        'wave5_phase_spacing_p3',
    ]

    baseline = rows[0]
    assert baseline['scenario'] == 'bringup'
    assert 'enable_lifecycle_observer:=true' in baseline['launch_args']
    assert 'fragmentation_staggered_enabled:=' not in baseline['launch_args']
    assert 'target_start_x_m:=-1500.0' in baseline['launch_args']
    assert 'target_start_y_m:=0.0' in baseline['launch_args']
    assert 'target_start_z_m:=300.0' in baseline['launch_args']

    expected_phase = ['0', '1', '2', '3']
    for row, phase in zip(rows[1:], expected_phase):
        assert row['scenario'] == 'bringup'
        assert 'fragmentation_staggered_enabled:=true' in row['launch_args']
        assert 'fragmentation_stagger_cycle_ticks:=7' in row['launch_args']
        assert 'fragmentation_stagger_gap_ticks:=4' in row['launch_args']
        assert f'fragmentation_stagger_phase_ticks:={phase}' in row['launch_args']
        assert 'target_start_x_m:=-1500.0' in row['launch_args']
        assert 'target_start_y_m:=0.0' in row['launch_args']
        assert 'target_start_z_m:=300.0' in row['launch_args']

    for row in rows:
        assert 'use_noisy_measurement:=true' in row['launch_args']
        assert 'noise_seed:=5101' in row['launch_args']
        assert 'sensor_input_topic:=/drone/position_noisy' in row['launch_args']
        assert 'enable_lifecycle_observer:=true' in row['launch_args']


def test_wave6_transferability_profiles_cover_geometry_matrix_without_new_mechanisms() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave6_transferability.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    geometries = {
        'g0_reference': ('-1500.0', '0.0', '300.0'),
        'g1_lateral_offset': ('-1500.0', '250.0', '300.0'),
        'g2_altitude_shift': ('-1500.0', '0.0', '450.0'),
        'g3_range_shift': ('-2000.0', '0.0', '300.0'),
    }
    roles = [
        'reachability_baseline',
        'phase3_bounded',
        'phase1_sentinel',
    ]

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        f'{geometry}_{role}'
        for geometry in geometries
        for role in roles
    ]

    for row in rows:
        profile_id = str(row['profile_id']).strip()
        launch_args = str(row['launch_args'])
        geometry_id = next(g for g in geometries if profile_id.startswith(g))
        x_m, y_m, z_m = geometries[geometry_id]

        assert row['scenario'] == 'bringup'
        assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in launch_args
        assert 'use_noisy_measurement:=true' in launch_args
        assert 'noise_seed:=5101' in launch_args
        assert 'sensor_input_topic:=/drone/position_noisy' in launch_args
        assert 'enable_lifecycle_observer:=true' in launch_args
        assert f'target_start_x_m:={x_m}' in launch_args
        assert f'target_start_y_m:={y_m}' in launch_args
        assert f'target_start_z_m:={z_m}' in launch_args

        if profile_id.endswith('reachability_baseline'):
            assert 'fragmentation_staggered_enabled:=' not in launch_args
            continue

        assert 'fragmentation_staggered_enabled:=true' in launch_args
        assert 'fragmentation_stagger_cycle_ticks:=7' in launch_args
        assert 'fragmentation_stagger_gap_ticks:=4' in launch_args
        if profile_id.endswith('phase3_bounded'):
            assert 'fragmentation_stagger_phase_ticks:=3' in launch_args
        elif profile_id.endswith('phase1_sentinel'):
            assert 'fragmentation_stagger_phase_ticks:=1' in launch_args
        else:
            raise AssertionError(f'unexpected Wave 6 profile id: {profile_id}')


def test_wave6_conservative_transfer_profiles_compare_prior_and_conservative_timing() -> None:
    path = (
        _REPO_ROOT
        / 'scripts'
        / 'evaluation'
        / 'fixtures'
        / 'ambiguity_sweep_profiles_wave6_conservative_transfer.csv'
    )
    rows = list(csv.DictReader(path.open(encoding='utf-8', newline='')))

    geometries = {
        'g0_reference': ('-1500.0', '0.0', '300.0'),
        'g1_lateral_offset': ('-1500.0', '250.0', '300.0'),
        'g2_altitude_shift': ('-1500.0', '0.0', '450.0'),
        'g3_range_shift': ('-2000.0', '0.0', '300.0'),
    }
    roles = [
        'reachability_baseline',
        'phase3_bounded',
        'conservative_fragmented',
    ]

    profile_ids = [str(row['profile_id']).strip() for row in rows]
    assert profile_ids == [
        f'{geometry}_{role}'
        for geometry in geometries
        for role in roles
    ]

    for row in rows:
        profile_id = str(row['profile_id']).strip()
        launch_args = str(row['launch_args'])
        geometry_id = next(g for g in geometries if profile_id.startswith(g))
        x_m, y_m, z_m = geometries[geometry_id]

        assert row['scenario'] == 'bringup'
        assert 'counter_uas_config:=config_gazebo_counter_uas.yaml' in launch_args
        assert 'use_noisy_measurement:=true' in launch_args
        assert 'noise_seed:=5101' in launch_args
        assert 'sensor_input_topic:=/drone/position_noisy' in launch_args
        assert 'enable_lifecycle_observer:=true' in launch_args
        assert f'target_start_x_m:={x_m}' in launch_args
        assert f'target_start_y_m:={y_m}' in launch_args
        assert f'target_start_z_m:={z_m}' in launch_args

        if profile_id.endswith('reachability_baseline'):
            assert 'fragmentation_staggered_enabled:=' not in launch_args
            continue

        assert 'fragmentation_staggered_enabled:=true' in launch_args
        if profile_id.endswith('phase3_bounded'):
            assert 'fragmentation_stagger_cycle_ticks:=7' in launch_args
            assert 'fragmentation_stagger_gap_ticks:=4' in launch_args
            assert 'fragmentation_stagger_phase_ticks:=3' in launch_args
        elif profile_id.endswith('conservative_fragmented'):
            assert 'fragmentation_stagger_cycle_ticks:=6' in launch_args
            assert 'fragmentation_stagger_gap_ticks:=3' in launch_args
            assert 'fragmentation_stagger_phase_ticks:=1' in launch_args
        else:
            raise AssertionError(f'unexpected Wave 6 conservative profile id: {profile_id}')
