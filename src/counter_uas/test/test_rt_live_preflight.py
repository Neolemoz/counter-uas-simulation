"""Tests for live runtime preflight (Web ↔ Gazebo Step 2)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.live_preflight import check_live_runtime_preflight  # noqa: E402


def test_live_preflight_schema() -> None:
    result = check_live_runtime_preflight()
    assert result["schema"] == "rt_live_runtime_preflight_v1"
    assert "checks" in result
    assert set(result["checks"]) == {
        "ros2_available",
        "gz_available",
        "rt_sandbox_gz_available",
    }
    if result["ok"]:
        assert result["blockers"] == []
    else:
        assert result["blockers"]


def test_live_preflight_fail_closed(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("rt_sandbox.live_preflight.shutil.which", lambda _: None)
    monkeypatch.setattr(
        "rt_sandbox.live_preflight._ros_pkg_available",
        lambda _pkg: False,
    )
    result = check_live_runtime_preflight()
    assert result["ok"] is False
    assert len(result["blockers"]) >= 2
