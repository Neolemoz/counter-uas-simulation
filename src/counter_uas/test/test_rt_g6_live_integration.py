"""Optional live Gazebo integration tests (PLAT-RT-G6)."""

from __future__ import annotations

import pytest

pytestmark = pytest.mark.g6_live


@pytest.mark.skip(reason="requires ros2, gz, and colcon install of rt_sandbox_gz")
def test_live_spawn_visible_in_gazebo() -> None:
    assert True
