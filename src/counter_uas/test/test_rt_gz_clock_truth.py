"""Clock truth and post-reset pause tests (PLAT-RT-LIVE-CONTROL-TRUTH1 Step 6)."""

from __future__ import annotations

import sys
import uuid
from pathlib import Path
from typing import Any

import pytest

_REPO = Path(__file__).resolve().parents[3]
_GZ_PKG = _REPO / "src" / "rt_sandbox_gz"
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
for path in (_GZ_PKG, _BRIDGE_PKG):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rt_sandbox.adapter_ipc import IpcRequest  # noqa: E402
from rt_sandbox.adapter_worker import AdapterWorker, MockSimState  # noqa: E402
from rt_sandbox.live_ros_client import LiveRosClient  # noqa: E402


def test_clock_mirror_includes_sim_time_when_present() -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live", paused=False)

    class _LiveRos:
        def sim_time_fields(self) -> dict[str, Any]:
            return {
                "sim_time_sec": 12,
                "sim_time_nsec": 500000000,
                "sim_time_source": "gazebo_clock_bridge",
            }

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

    worker._state.live_ros = _LiveRos()  # type: ignore[assignment]
    payload = worker._clock_mirror_payload()
    assert payload["paused"] is False
    assert payload["mode"] == "live"
    assert payload["sim_time_sec"] == 12
    assert payload["sim_time_nsec"] == 500000000
    assert payload["sim_time_source"] == "gazebo_clock_bridge"


def test_clock_mirror_without_gazebo_clock_keeps_legacy_fields() -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live", paused=True)

    class _LiveRos:
        def sim_time_fields(self) -> dict[str, Any]:
            return {}

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

    worker._state.live_ros = _LiveRos()  # type: ignore[assignment]
    payload = worker._clock_mirror_payload()
    assert payload == {"paused": True, "mode": "live"}
    assert "sim_time_sec" not in payload


def test_live_ros_client_sim_time_fields_empty_by_default() -> None:
    client = LiveRosClient(str(uuid.uuid4()))
    assert client.sim_time_fields() == {}


def test_live_ros_client_sim_time_fields_when_set() -> None:
    client = LiveRosClient(str(uuid.uuid4()))
    client._sim_time_sec = 3
    client._sim_time_nsec = 250
    fields = client.sim_time_fields()
    assert fields["sim_time_sec"] == 3
    assert fields["sim_time_nsec"] == 250
    assert fields["sim_time_source"] == "gazebo_clock_bridge"


def test_reset_while_paused_reapplies_gazebo_pause(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live", paused=True, world_name="rt_sandbox_flat")
    calls: list[str] = []

    class _LiveRos:
        def publish_pose_cmd(self, **kwargs: Any) -> None:
            pass

        def publish_world_reset(self) -> None:
            pass

        def clear_state_cache(self) -> None:
            pass

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

    worker._state.live_ros = _LiveRos()  # type: ignore[assignment]

    class _Mod:
        @staticmethod
        def gz_world_reset_all(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append(f"reset:{world_name}")
            return True

        @staticmethod
        def gz_world_pause(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append(f"pause:{world_name}")
            return True

        @staticmethod
        def gz_world_resume(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append("resume")
            return True

    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: "/usr/bin/gz")
    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: _Mod())

    out = worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    assert out.ok is True
    assert calls == ["reset:rt_sandbox_flat", "pause:rt_sandbox_flat"]
    assert worker._state.paused is True
    mirror = worker._clock_mirror_payload()
    assert mirror["paused"] is True


def test_reset_while_running_does_not_force_pause(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live", paused=False, world_name="rt_sandbox_flat")
    calls: list[str] = []

    class _LiveRos:
        def publish_pose_cmd(self, **kwargs: Any) -> None:
            pass

        def publish_world_reset(self) -> None:
            pass

        def clear_state_cache(self) -> None:
            pass

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

    worker._state.live_ros = _LiveRos()  # type: ignore[assignment]

    class _Mod:
        @staticmethod
        def gz_world_reset_all(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append("reset")
            return True

        @staticmethod
        def gz_world_pause(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append("pause")
            return True

    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: "/usr/bin/gz")
    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: _Mod())

    worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    assert calls == ["reset"]
    assert worker._state.paused is False


def test_live_control_truth_contract_doc_mentions_reset_world() -> None:
    doc = (_REPO / "docs/evaluation/rt_live_control_truth_v1.md").read_text(encoding="utf-8")
    assert "reset_world" in doc
    assert "gz_world_pause" in doc or "WorldControl" in doc
    assert "sim_time_sec" in doc
    assert "reset may resume" in doc.lower() or "re-apply" in doc.lower()
