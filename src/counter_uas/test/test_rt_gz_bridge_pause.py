"""Bridge-node pause truth tests (PLAT-RT-LIVE-CONTROL-TRUTH1 Step 4)."""

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
from rt_sandbox.kinematic_plant import AeroEnvironment, KinematicLimits  # noqa: E402
from rt_sandbox_gz.gz_bridge_node import RtSandboxGzBridgeNode  # noqa: E402


def _limits() -> KinematicLimits:
    return KinematicLimits(
        max_speed_mps=25.0,
        max_accel_mps2=30.0,
        max_turn_rate_rad_s=2.5,
        max_climb_mps=8.0,
    )


def _bridge_stub(*, paused: bool = False) -> RtSandboxGzBridgeNode:
    node = RtSandboxGzBridgeNode.__new__(RtSandboxGzBridgeNode)
    node._paused = paused
    node._plant_enabled = True
    node._world = "rt_sandbox_flat"
    node._ground_snap = False
    node._limits = _limits()
    node._aero = AeroEnvironment()
    node._sync_seq = 0
    node._entities = {}
    node._model_paths = {"drone": "/fake/model.sdf"}
    node._state_pub = MagicMock()
    node.get_logger = lambda: logging.getLogger("test_rt_gz_bridge_pause")
    return node


def _drone_entity(*, x: float = 0.0, y: float = 0.0) -> dict[str, Any]:
    commanded = {"x": x, "y": y, "z": 0.0, "yaw_deg": 0.0}
    ent = {
        "entity_id": "ent-1",
        "entity_type": "drone",
        "sim_entity_ref": "sim-drone",
        **default_entity_record("drone", commanded),
    }
    ent["commanded_pose"] = {"x": 100.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    return ent


def test_worker_publishes_clock_mirror_on_pause_and_resume() -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="live")

    class _LiveRos:
        payloads: list[dict[str, Any]] = []

        def publish_clock(self, payload: dict[str, Any]) -> None:
            self.payloads.append(dict(payload))

    live = _LiveRos()
    worker._state.live_ros = live  # type: ignore[assignment]

    worker._pause(IpcRequest(op="pause", request_id=str(uuid.uuid4()), session_id=sid))
    assert live.payloads[-1] == {"paused": True, "mode": "live"}
    assert worker._state.published[-1]["payload"] == {"paused": True, "mode": "live"}

    worker._resume(IpcRequest(op="resume", request_id=str(uuid.uuid4()), session_id=sid))
    assert live.payloads[-1] == {"paused": False, "mode": "live"}


def test_bridge_node_timer_skips_integration_when_paused(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _bridge_stub(paused=True)
    ent = _drone_entity()
    node._entities["ent-1"] = ent
    pose_push: list[str] = []
    monkeypatch.setattr(
        node,
        "_push_pose_to_gazebo",
        lambda e: pose_push.append(str(e.get("sim_entity_ref"))) or True,
    )
    monkeypatch.setattr(node, "_publish_state", lambda: None)

    node._on_timer()
    assert pose_push == []
    assert ent["pose"]["x"] == 0.0


def test_bridge_node_resume_resets_dt_without_jump(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _bridge_stub(paused=False)
    ent = _drone_entity()
    ent["last_integrate_monotonic"] = 100.0
    node._entities["ent-1"] = ent
    clock = {"t": 200.0}

    monkeypatch.setattr(time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(node, "_push_pose_to_gazebo", lambda _e: True)
    monkeypatch.setattr(node, "_publish_state", lambda: None)

    node._on_clock(_clock_msg({"paused": True}))
    node._on_timer()
    assert ent["pose"]["x"] == 0.0

    node._on_clock(_clock_msg({"paused": False}))
    assert ent["last_integrate_monotonic"] == 200.0

    clock["t"] = 200.05
    before = ent["pose"]["x"]
    node._on_timer()
    after = ent["pose"]["x"]
    assert after > before
    assert after < 50.0


def test_bridge_node_explicit_move_while_paused_snaps(monkeypatch: pytest.MonkeyPatch) -> None:
    node = _bridge_stub(paused=True)
    ent = _drone_entity()
    node._entities["ent-1"] = ent
    pushed: list[dict[str, float]] = []

    def _push(e: dict[str, Any]) -> bool:
        pushed.append(dict(e["pose"]))
        return True

    monkeypatch.setattr(node, "_push_pose_to_gazebo", _push)
    monkeypatch.setattr(node, "_publish_state", lambda: None)
    monkeypatch.setattr("rt_sandbox_gz.gz_bridge_node.gz_service", lambda *a, **k: True)

    node._apply_entity(
        "ent-1",
        {
            "entity_type": "drone",
            "sim_entity_ref": "sim-drone",
            "pose": {"x": 42.0, "y": 7.0, "z": 0.0, "yaw_deg": 0.0},
        },
    )
    assert pushed == [{"x": 42.0, "y": 7.0, "z": 0.0, "yaw_deg": 0.0}]
    assert ent["commanded_pose"]["x"] == 42.0


def test_mock_poll_telemetry_does_not_drift_while_paused(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    sid = str(uuid.uuid4())
    worker._state = MockSimState(session_id=sid, mode="mock", paused=False)
    worker._state.kinematic_plant_enabled = True
    worker._state.entities["ent-1"] = default_entity_record(
        "drone",
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
    )
    worker._state.entities["ent-1"]["commanded_pose"] = {
        "x": 100.0,
        "y": 0.0,
        "z": 0.0,
        "yaw_deg": 0.0,
    }
    worker._state.last_poll_monotonic = 1000.0

    times = iter([1001.0, 1002.0, 1003.0, 1004.0, 1005.0, 1006.0])
    monkeypatch.setattr(time, "monotonic", lambda: next(times))

    req = lambda: IpcRequest(op="poll_telemetry", request_id=str(uuid.uuid4()), session_id=sid)
    first = worker._poll_telemetry(req())
    x_running = first.result["entity_pose_mirror"]["entities"][0]["pose"]["x"]
    assert x_running > 0.0

    worker._state.paused = True
    paused_pose = worker._poll_telemetry(req()).result["entity_pose_mirror"]["entities"][0]["pose"]["x"]
    assert paused_pose == x_running
    worker._poll_telemetry(req())
    assert worker._poll_telemetry(req()).result["entity_pose_mirror"]["entities"][0]["pose"]["x"] == x_running

    worker._state.paused = False
    worker._reset_mock_poll_clock()
    resumed = worker._poll_telemetry(req()).result["entity_pose_mirror"]["entities"][0]["pose"]["x"]
    assert resumed > paused_pose
    assert resumed - paused_pose <= x_running * 1.5


def _clock_msg(payload: dict[str, Any]) -> Any:
    msg = MagicMock()
    msg.data = json.dumps(payload)
    return msg
