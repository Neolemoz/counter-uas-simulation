"""Reset truth tests (PLAT-RT-LIVE-CONTROL-TRUTH1 Step 5)."""

from __future__ import annotations

import json
import logging
import sys
import time
import uuid
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import pytest

_REPO = Path(__file__).resolve().parents[3]
_GZ_PKG = _REPO / "src" / "rt_sandbox_gz"
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
for path in (_GZ_PKG, _BRIDGE_PKG):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rt_sandbox.adapter_ipc import IpcRequest  # noqa: E402
from rt_sandbox.adapter_worker import AdapterWorker, MockSimState  # noqa: E402
from rt_sandbox.kinematic_entity import default_entity_record  # noqa: E402
from rt_sandbox_gz.gz_bridge_node import RtSandboxGzBridgeNode  # noqa: E402


def _bridge_stub() -> RtSandboxGzBridgeNode:
    node = RtSandboxGzBridgeNode.__new__(RtSandboxGzBridgeNode)
    node._paused = False
    node._plant_enabled = True
    node._world = "rt_sandbox_flat"
    node._entities = {
        "ent-1": {
            "entity_id": "ent-1",
            "entity_type": "drone",
            "sim_entity_ref": "sim-drone",
            "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0},
            "commanded_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0},
            "velocity": {"x": 3.0, "y": 0.0, "z": 0.0},
            "last_integrate_monotonic": 100.0,
        }
    }
    node._sync_seq = 7
    node._state_pub = MagicMock()
    node.get_logger = lambda: logging.getLogger("test_rt_gz_reset")
    return node


def test_live_reset_calls_gz_world_reset_all(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live", world_name="rt_sandbox_flat")
    worker._state.sim_entity_refs["ent-1"] = "sim-drone"
    calls: list[str] = []

    class _LiveRos:
        deletes: list[str] = []
        reset = False
        cleared = False

        def publish_pose_cmd(self, **kwargs: Any) -> None:
            self.deletes.append(str(kwargs.get("sim_entity_ref")))

        def publish_world_reset(self) -> None:
            self.reset = True

        def clear_state_cache(self) -> None:
            self.cleared = True

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

    live = _LiveRos()
    worker._state.live_ros = live  # type: ignore[assignment]

    class _Mod:
        @staticmethod
        def gz_world_reset_all(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append(world_name)
            return True

    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: "/usr/bin/gz")
    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: _Mod())

    out = worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    assert out.ok is True
    assert out.result == {"reset": True}
    assert calls == ["rt_sandbox_flat"]
    assert live.deletes == ["sim-drone"]
    assert live.reset is True
    assert live.cleared is True


def test_reset_clears_adapter_maps() -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="mock", paused=True)
    worker._state.entities["ent-1"] = default_entity_record(
        "drone",
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
    )
    worker._state.drift_offsets["ent-1"] = {"x": 1.0}
    worker._state.sim_entity_refs["ent-1"] = "sim-drone"
    worker._state.sync_seq = 5
    worker._state.telemetry_seq = 9
    worker._state.last_poll_monotonic = 123.0
    worker._state.published.append({"topic": "/clock", "payload": {"paused": True}})

    out = worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    assert out.ok is True
    assert worker._state.entities == {}
    assert worker._state.drift_offsets == {}
    assert worker._state.sim_entity_refs == {}
    assert worker._state.sync_seq == 0
    assert worker._state.telemetry_seq == 0
    assert worker._state.last_poll_monotonic is None
    assert worker._state.paused is True
    assert worker._state.published[-1]["payload"] == {"paused": True, "mode": "mock"}


def test_bridge_reset_world_clears_state_and_publishes_empty() -> None:
    node = _bridge_stub()
    node._reset_runtime_state()
    assert node._entities == {}
    assert node._sync_seq == 0
    assert node._state_pub.publish.call_count == 1
    payload = json.loads(node._state_pub.publish.call_args[0][0].data)
    assert payload["schema"] == "rt_entity_state_v1"
    assert payload["entities"] == []


def test_bridge_on_cmd_reset_world_op() -> None:
    node = _bridge_stub()
    msg = MagicMock()
    msg.data = json.dumps(
        {
            "schema": "rt_entity_pose_cmd_v1",
            "op": "reset_world",
            "entity_id": "_world_",
            "entity_type": "drone",
            "pose": {},
        }
    )
    node._on_cmd(msg)
    assert node._entities == {}
    assert node._sync_seq == 0


def test_reset_does_not_retain_stale_live_feedback(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live")
    worker._state.sim_entity_refs["ent-1"] = "sim-drone"

    class _LiveRos:
        def publish_pose_cmd(self, **kwargs: Any) -> None:
            pass

        def publish_world_reset(self) -> None:
            pass

        def clear_state_cache(self) -> None:
            pass

        def publish_clock(self, payload: dict[str, Any]) -> None:
            pass

        def get_feedback(self) -> dict[str, Any]:
            return {"entities": [], "sync_seq": 0}

    live = _LiveRos()
    worker._state.live_ros = live  # type: ignore[assignment]
    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: None)
    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: None)

    worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    poll = worker._poll_feedback(IpcRequest(op="poll_feedback", request_id=str(uuid.uuid4()), session_id=sid))
    assert poll.ok is True
    assert poll.result["entities"] == []


def test_mock_reset_telemetry_has_no_stale_entities(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="mock")
    worker._state.entities["ent-1"] = default_entity_record(
        "drone",
        {"x": 10.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
    )
    worker._state.entities["ent-1"]["commanded_pose"] = {
        "x": 100.0,
        "y": 0.0,
        "z": 0.0,
        "yaw_deg": 0.0,
    }
    worker._state.last_poll_monotonic = 1000.0
    monkeypatch.setattr(time, "monotonic", lambda: 1001.0)

    before = worker._poll_telemetry(
        IpcRequest(op="poll_telemetry", request_id=str(uuid.uuid4()), session_id=sid)
    )
    assert len(before.result["entity_pose_mirror"]["entities"]) == 1

    worker._reset_world(IpcRequest(op="reset_world", request_id=str(uuid.uuid4()), session_id=sid))
    after = worker._poll_telemetry(
        IpcRequest(op="poll_telemetry", request_id=str(uuid.uuid4()), session_id=sid)
    )
    assert after.result["entity_pose_mirror"]["entities"] == []
