from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_run_capture():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'run_capture.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('run_capture', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_run_capture_supports_bringup_topology_explicitly() -> None:
    mod = _load_run_capture()
    assert mod._ros_launch_cmd_for_scenario('single') == 'ros2 launch gazebo_target_sim gazebo_target.launch.py'
    assert mod._ros_launch_cmd_for_scenario('multi') == 'ros2 launch gazebo_target_sim gazebo_target_multi.launch.py'
    assert mod._ros_launch_cmd_for_scenario('bringup') == 'ros2 launch counter_uas bringup.launch.py'


def test_monte_carlo_accepts_bringup_scenario() -> None:
    source = (_REPO_ROOT / 'scripts' / 'monte_carlo.py').read_text(encoding='utf-8')
    assert 'choices=["single", "multi", "bringup"]' in source
