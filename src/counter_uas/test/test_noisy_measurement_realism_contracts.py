from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_noisy_measurement_realism_source_declares_additive_optional_knobs() -> None:
    source = (
        _REPO_ROOT
        / 'src'
        / 'gazebo_target_sim'
        / 'gazebo_target_sim'
        / 'noisy_measurement_node.py'
    ).read_text(encoding='utf-8')

    assert 'self.declare_parameter("delay_mean_s", 0.0)' in source
    assert 'self.declare_parameter("delay_jitter_s", 0.0)' in source
    assert 'self.declare_parameter("publish_period_jitter_s", 0.0)' in source
    assert 'self.declare_parameter("burst_dropout_prob", 0.0)' in source
    assert 'self.declare_parameter("stale_detection_enabled", False)' in source
    assert 'self.declare_parameter("ghost_detection_prob", 0.0)' in source
    assert 'self.declare_parameter("ghost_placement_mode", "broad")' in source
    assert 'self.declare_parameter("ghost_near_threshold_xy_min_m", 18.0)' in source
    assert 'self.declare_parameter("ghost_near_threshold_xy_max_m", 24.0)' in source
    assert 'self.declare_parameter("ghost_persistence_ticks", 1)' in source
    assert 'self.declare_parameter("fragmentation_prob", 0.0)' in source
    assert 'self.declare_parameter("fragmentation_staggered_enabled", False)' in source
    assert 'self.declare_parameter("fragmentation_stagger_cycle_ticks", 6)' in source
    assert 'self.declare_parameter("fragmentation_stagger_phase_ticks", 1)' in source
    assert 'self.declare_parameter("fragmentation_stagger_gap_ticks", 2)' in source
    assert "'[REALISM_EVENT] delayed_detection '" in source
    assert "'[REALISM_EVENT] stale_detection '" in source
    assert "'[REALISM_EVENT] dropout_burst_start '" in source
    assert "'[REALISM_EVENT] ghost_detection '" in source
    assert "'[REALISM_EVENT] fragmented_gap_start '" in source
    assert "'[REALISM_EVENT] fragmented_gap_end '" in source
    assert "placement={self._ghost_placement_mode}" in source
    assert "persistence_ticks={self._ghost_persistence_ticks}" in source
    assert "persistence_remaining={persistence_remaining}" in source
    assert "tick_index={self._tick_index}" in source
    assert "seed={self._seed}" in source
    assert "mode=staggered" in source
    assert "mode=random" in source


def test_launch_files_keep_realism_toggles_disabled_by_default() -> None:
    single = (
        _REPO_ROOT
        / 'src'
        / 'gazebo_target_sim'
        / 'launch'
        / 'gazebo_target.launch.py'
    ).read_text(encoding='utf-8')
    multi = (
        _REPO_ROOT
        / 'src'
        / 'gazebo_target_sim'
        / 'launch'
        / 'gazebo_target_multi.launch.py'
    ).read_text(encoding='utf-8')

    for source in (single, multi):
        assert "DeclareLaunchArgument(\n                'noise_delay_mean_s',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'noise_delay_jitter_s',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'noise_period_jitter_s',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'stale_detection_enabled',\n                default_value='false'" in source
        assert "DeclareLaunchArgument(\n                'stale_max_consecutive',\n                default_value='0'" in source
        assert "DeclareLaunchArgument(\n                'burst_dropout_prob',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'ghost_detection_prob',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'ghost_placement_mode',\n                default_value='broad'" in source
        assert "DeclareLaunchArgument(\n                'ghost_near_threshold_xy_min_m',\n                default_value='18.0'" in source
        assert "DeclareLaunchArgument(\n                'ghost_near_threshold_xy_max_m',\n                default_value='24.0'" in source
        assert "DeclareLaunchArgument(\n                'ghost_persistence_ticks',\n                default_value='1'" in source
        assert "DeclareLaunchArgument(\n                'fragmentation_prob',\n                default_value='0.0'" in source
        assert "DeclareLaunchArgument(\n                'fragmentation_staggered_enabled',\n                default_value='false'" in source
        assert "DeclareLaunchArgument(\n                'fragmentation_stagger_cycle_ticks',\n                default_value='6'" in source
        assert "DeclareLaunchArgument(\n                'fragmentation_stagger_phase_ticks',\n                default_value='1'" in source
        assert "DeclareLaunchArgument(\n                'fragmentation_stagger_gap_ticks',\n                default_value='2'" in source


def test_bringup_launch_keeps_wave2_propagation_disabled_by_default() -> None:
    source = (
        _REPO_ROOT
        / 'src'
        / 'counter_uas'
        / 'launch'
        / 'bringup.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n                'use_noisy_measurement',\n                default_value='false'" in source
    assert "DeclareLaunchArgument(\n                'sensor_input_topic',\n                default_value='/drone/position'" in source
    assert "DeclareLaunchArgument(\n                'ghost_persistence_ticks',\n                default_value='1'" in source
    assert "DeclareLaunchArgument(\n                'fragmentation_stagger_gap_ticks',\n                default_value='2'" in source
    assert "remappings=[('/drone/position', LaunchConfiguration('sensor_input_topic'))]" in source
