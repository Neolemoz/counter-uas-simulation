"""Regression checks for delayed Gazebo pause timer lifecycle.

The hit path must not create an anonymous periodic rclpy timer.  If that timer
survives a Gazebo reset, a later tick can pause the next run unexpectedly.
"""

from __future__ import annotations

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_NODE = _REPO_ROOT / 'src' / 'gazebo_target_sim' / 'gazebo_target_sim' / 'interception_logic_node.py'


def _source() -> str:
    assert _NODE.is_file(), _NODE
    return _NODE.read_text(encoding='utf-8')


def test_gazebo_pause_timer_is_one_shot_and_cancellable() -> None:
    src = _source()

    assert 'self._pause_gz_timer = None' in src
    assert 'def _cancel_gazebo_pause_timer(self) -> None:' in src
    assert 'def _on_gazebo_pause_timer(self) -> None:' in src
    assert 'self._pause_gz_timer = self.create_timer(3.5, self._on_gazebo_pause_timer)' in src
    assert 'self._cancel_gazebo_pause_timer()\n        self._pause_gazebo_world()' in src


def test_reset_cancels_pending_gazebo_pause_timer() -> None:
    src = _source()
    reset_start = src.index('    def _on_gz_sim_reset(self) -> None:')
    reset_end = src.index('    def _cancel_stop_signal_repeat_timer(self) -> None:')
    reset_body = src[reset_start:reset_end]

    assert 'self._gz_pause_sent = False' in reset_body
    assert 'self._cancel_gazebo_pause_timer()' in reset_body


def test_hit_paths_use_shared_pause_scheduler() -> None:
    src = _source()

    assert 'self.create_timer(3.5, self._pause_gazebo_world)' not in src
    assert 'if self._pause_gz_on_hit:\n                self._pause_gazebo_world()' not in src
    assert src.count('self._schedule_gazebo_pause()') >= 2
