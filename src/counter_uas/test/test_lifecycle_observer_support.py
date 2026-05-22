from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_bringup_launch_can_enable_passive_lifecycle_observer_default_off() -> None:
    source = (
        _REPO_ROOT
        / 'src'
        / 'counter_uas'
        / 'launch'
        / 'bringup.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n                'enable_lifecycle_observer',\n                default_value='false'" in source
    assert "condition=IfCondition(LaunchConfiguration('enable_lifecycle_observer'))" in source
    assert "package='counter_uas'" in source
    assert "executable='lifecycle_observer_node'" in source


def test_lifecycle_observer_source_stays_passive_and_grepable() -> None:
    source = (
        _REPO_ROOT
        / 'src'
        / 'counter_uas'
        / 'counter_uas'
        / 'lifecycle_observer_node.py'
    ).read_text(encoding='utf-8')

    assert "create_subscription(Odometry, self._tracks_state_topic" in source
    assert "create_subscription(String, self._selected_id_topic" in source
    assert "'[LIFECYCLE_OBSERVER]'" in source
    assert "'[TRACK_CONTINUITY]'" in source
    assert "'[SELECTION_PROXY]'" in source
    assert "'[TRACK_PERSISTENCE]'" in source
    assert 'create_publisher' not in source
    assert 'publish(' not in source
