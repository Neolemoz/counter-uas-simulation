"""Adapter worker subprocess — mock or live Gazebo/ROS (PLAT-RT-G2).

Reads JSON-lines requests on stdin, writes responses on stdout.
Bridge process must not import rclpy.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()

from rt_sandbox.adapter_ipc import IpcRequest, IpcResponse
from rt_sandbox.kinematic_entity import (
    advance_entity_toward_command,
    default_entity_record,
    entity_telemetry_fields,
)
from rt_sandbox.engagement_limits import default_engagement_limits
from rt_sandbox.kinematic_plant import AeroEnvironment, KinematicLimits
from rt_sandbox.live_ros_client import LiveRosClient
from rt_sandbox.ros_allowlist import (
    allowed_session_topics,
    classify_topic,
    session_topic_prefix,
)


@dataclass
class MockSimState:
    session_id: str
    mode: str
    paused: bool = False
    entities: dict[str, dict[str, Any]] = field(default_factory=dict)
    published: list[dict[str, Any]] = field(default_factory=list)
    sim_entity_refs: dict[str, str] = field(default_factory=dict)
    drift_offsets: dict[str, dict[str, float]] = field(default_factory=dict)
    sync_seq: int = 0
    telemetry_seq: int = 0
    gazebo_pid: int | None = None
    launch_proc: subprocess.Popen[bytes] | None = None
    live_ros: LiveRosClient | None = None
    ros_domain_id: int | None = None
    ground_snap_enabled: bool = True
    enable_fidelity_coupling: bool = False
    fidelity_ground_z_m: float = 0.0
    kinematic_plant_enabled: bool = True
    kinematic_limits: KinematicLimits = field(
        default_factory=lambda: KinematicLimits(
            **default_engagement_limits().kinematic_limits_kwargs()
        )
    )
    aero: AeroEnvironment = field(
        default_factory=lambda: AeroEnvironment(**default_engagement_limits().aero_kwargs())
    )
    last_poll_monotonic: float | None = None

    def feedback_pose_for(self, entity_id: str) -> dict[str, float]:
        ent = self.entities.get(entity_id)
        if ent is None:
            return {}
        pose = dict(ent["pose"])
        offset = self.drift_offsets.get(entity_id)
        if offset:
            for k, v in offset.items():
                if k in pose:
                    pose[k] = float(pose[k]) + float(v)
        return pose

    def clock_payload(self) -> dict[str, Any]:
        return {"paused": self.paused, "mode": self.mode}

    def entity_telemetry_fields_for(
        self,
        entity_id: str,
    ) -> dict[str, Any]:
        ent = self.entities.get(entity_id) or {}
        pose = self.feedback_pose_for(entity_id) or dict(ent.get("pose") or {})
        telem = entity_telemetry_fields(ent)
        return {
            "position": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "z": float(pose.get("z", 0.0)),
            },
            **telem,
            "target_state": "none",
            "lifecycle_state": "spawned",
        }

    def entity_state_payload(self) -> dict[str, Any]:
        return {
            "entities": [
                {
                    "entity_id": eid,
                    "entity_type": ent["entity_type"],
                    "pose": dict(ent["pose"]),
                    "sim_entity_ref": self.sim_entity_refs.get(eid),
                    **self.entity_telemetry_fields_for(eid),
                }
                for eid, ent in self.entities.items()
            ]
        }


class AdapterWorker:
    def __init__(self) -> None:
        self._state: MockSimState | None = None

    def run_loop(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
                req = IpcRequest(
                    op=str(data["op"]),
                    request_id=str(data.get("request_id", "")),
                    session_id=str(data.get("session_id", "")),
                    payload=dict(data.get("payload") or {}),
                )
                resp = self.handle(req)
            except Exception as exc:  # noqa: BLE001 — worker boundary
                rid = ""
                try:
                    rid = str(data.get("request_id", ""))
                except NameError:
                    pass
                resp = IpcResponse(
                    request_id=rid,
                    ok=False,
                    error_code="RUNTIME_UNAVAILABLE",
                    error_message=str(exc),
                )
            sys.stdout.write(resp.to_line())
            sys.stdout.flush()

    def handle(self, req: IpcRequest) -> IpcResponse:
        handlers = {
            "attach": self._attach,
            "detach": self._detach,
            "health": self._health,
            "pause": self._pause,
            "resume": self._resume,
            "apply_pose": self._apply_pose,
            "delete_entity": self._delete_entity,
            "reset_world": self._reset_world,
            "validate_topic": self._validate_topic,
            "mock_publish": self._mock_publish,
            "poll_feedback": self._poll_feedback,
            "mock_inject_drift": self._mock_inject_drift,
            "resync_all": self._resync_all,
            "poll_telemetry": self._poll_telemetry,
        }
        fn = handlers.get(req.op)
        if fn is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="COMMAND_FORBIDDEN",
                error_message=f"unknown op {req.op}",
            )
        return fn(req)

    def _attach(self, req: IpcRequest) -> IpcResponse:
        mode = str(req.payload.get("mode", "mock"))
        if mode not in {"mock", "live"}:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="COMMAND_FORBIDDEN",
                error_message="invalid adapter mode",
            )
        self._state = MockSimState(session_id=req.session_id, mode=mode)
        ros_domain_id = req.payload.get("ros_domain_id")
        if ros_domain_id is not None:
            self._state.ros_domain_id = int(ros_domain_id)
        ground_snap = req.payload.get("ground_snap_enabled")
        if ground_snap is not None:
            self._state.ground_snap_enabled = bool(ground_snap)
        coupling = req.payload.get("enable_fidelity_coupling")
        if coupling is not None:
            self._state.enable_fidelity_coupling = bool(coupling)
        ground_z = req.payload.get("fidelity_ground_z_m")
        if ground_z is not None:
            self._state.fidelity_ground_z_m = float(ground_z)
        plant_enabled = req.payload.get("kinematic_plant_enabled")
        if plant_enabled is not None:
            self._state.kinematic_plant_enabled = bool(plant_enabled)
        limits_in = req.payload.get("kinematic_limits")
        if isinstance(limits_in, dict):
            self._state.kinematic_limits = KinematicLimits.from_mapping(limits_in)
        aero_in = req.payload.get("aero")
        if isinstance(aero_in, dict):
            self._state.aero = AeroEnvironment.from_mapping(aero_in)
        elif req.payload.get("drag_decel_per_mps") is not None or req.payload.get("wind_x_mps") is not None:
            self._state.aero = AeroEnvironment.from_mapping(req.payload)
        gazebo_pid = None
        if mode == "live":
            gazebo_pid, err = self._try_launch_gazebo(req)
            if err:
                self._state = None
                return IpcResponse(
                    request_id=req.request_id,
                    ok=False,
                    error_code="RUNTIME_UNAVAILABLE",
                    error_message=err,
                )
            self._state.gazebo_pid = gazebo_pid
            client = LiveRosClient(
                req.session_id,
                ros_domain_id=self._state.ros_domain_id,
                ground_snap_enabled=self._state.ground_snap_enabled,
            )
            if client.start():
                self._state.live_ros = client
            else:
                self._teardown_launch()
                self._state = None
                return IpcResponse(
                    request_id=req.request_id,
                    ok=False,
                    error_code="RUNTIME_UNAVAILABLE",
                    error_message="rclpy unavailable for live adapter",
                )
        topics = sorted(allowed_session_topics(req.session_id))
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={
                "attached": True,
                "mode": mode,
                "worker_pid": os.getpid(),
                "gazebo_pid": gazebo_pid,
                "topics": topics,
                "prefix": session_topic_prefix(req.session_id),
            },
        )

    def _try_launch_gazebo(self, req: IpcRequest) -> tuple[int | None, str | None]:
        if shutil.which("ros2") is None:
            return None, "ros2 not found on PATH"
        ros_domain_id = req.payload.get("ros_domain_id")
        env = os.environ.copy()
        if ros_domain_id is not None:
            env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        cmd = [
            "ros2",
            "launch",
            "rt_sandbox_gz",
            "rt_sandbox.launch.py",
            f"session_id:={req.session_id}",
            "use_gazebo_gui:=false",
        ]
        world = req.payload.get("rt_sandbox_world")
        if world:
            cmd.append(f"world_name:={world}")
        ground_snap = req.payload.get("ground_snap_enabled")
        if ground_snap is not None:
            cmd.append(f"ground_snap_enabled:={'true' if ground_snap else 'false'}")
        assert self._state is not None
        cmd.append(
            f"kinematic_plant_enabled:={'true' if self._state.kinematic_plant_enabled else 'false'}"
        )
        lim = self._state.kinematic_limits
        cmd.append(f"max_speed_mps:={lim.max_speed_mps}")
        cmd.append(f"max_accel_mps2:={lim.max_accel_mps2}")
        cmd.append(f"max_turn_rate_rad_s:={lim.max_turn_rate_rad_s}")
        cmd.append(f"max_climb_mps:={lim.max_climb_mps}")
        aero = self._state.aero
        cmd.append(f"drag_decel_per_mps:={aero.drag_decel_per_mps}")
        cmd.append(f"wind_x_mps:={aero.wind_x_mps}")
        cmd.append(f"wind_y_mps:={aero.wind_y_mps}")
        cmd.append(f"wind_z_mps:={aero.wind_z_mps}")
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
                start_new_session=True,
            )
        except OSError as exc:
            return None, str(exc)
        assert self._state is not None
        self._state.launch_proc = proc
        ready_timeout_s = float(req.payload.get("ready_timeout_s") or 60.0)
        err = self._wait_for_launch_ready(req, proc, ready_timeout_s)
        if err:
            self._teardown_launch()
            return None, err
        return proc.pid, None

    def _wait_for_launch_ready(
        self,
        req: IpcRequest,
        proc: subprocess.Popen[bytes],
        timeout_s: float,
    ) -> str | None:
        state_topic = f"{session_topic_prefix(req.session_id)}entity_state"
        ros_domain_id = req.payload.get("ros_domain_id")
        env = os.environ.copy()
        if ros_domain_id is not None:
            env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        deadline = time.monotonic() + max(0.5, timeout_s)
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return "gazebo launch exited during readiness wait"
            if shutil.which("ros2") is not None:
                try:
                    listed = subprocess.run(
                        ["ros2", "topic", "list"],
                        capture_output=True,
                        text=True,
                        timeout=5,
                        env=env,
                    )
                    if listed.returncode == 0 and state_topic in listed.stdout:
                        return None
                except (OSError, subprocess.TimeoutExpired):
                    pass
            time.sleep(0.5)
        if proc.poll() is not None:
            return "gazebo launch exited during readiness wait"
        return None

    def _detach(self, req: IpcRequest) -> IpcResponse:
        self._teardown_launch()
        self._state = None
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"detached": True},
        )

    def _shutdown_live_ros(self) -> None:
        if self._state is None or self._state.live_ros is None:
            return
        self._state.live_ros.shutdown()
        self._state.live_ros = None

    def _teardown_launch(self) -> None:
        self._shutdown_live_ros()
        if self._state is None:
            return
        proc = self._state.launch_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=2)
        self._state.launch_proc = None
        self._state.gazebo_pid = None

    def _health(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        alive = True
        if self._state.launch_proc is not None:
            alive = self._state.launch_proc.poll() is None
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={
                "alive": alive,
                "mode": self._state.mode,
                "paused": self._state.paused,
                "entity_count": len(self._state.entities),
            },
        )

    def _pause(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        self._state.paused = True
        self._record_clock_topic()
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"paused": True},
        )

    def _resume(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        self._state.paused = False
        self._record_clock_topic()
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"paused": False},
        )

    def _apply_pose(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        entity_id = str(req.payload["entity_id"])
        entity_type = str(req.payload["entity_type"])
        pose = dict(req.payload["pose"])
        sim_ref = self._state.sim_entity_refs.get(entity_id)
        if sim_ref is None:
            sim_ref = f"sim-{uuid.uuid4().hex[:8]}"
            self._state.sim_entity_refs[entity_id] = sim_ref
        existing = self._state.entities.get(entity_id)
        if existing is None:
            self._state.entities[entity_id] = default_entity_record(entity_type, pose)
        else:
            existing["entity_type"] = entity_type
        ent = self._state.entities[entity_id]
        if self._state.mode != "live":
            advance_entity_toward_command(
                ent,
                pose,
                limits=self._state.kinematic_limits,
                plant_enabled=self._state.kinematic_plant_enabled,
                aero=self._state.aero,
            )
        else:
            ent["commanded_pose"] = dict(pose)
            if not self._state.kinematic_plant_enabled:
                ent["pose"] = dict(pose)
        self._state.sync_seq += 1
        sync_seq = self._state.sync_seq
        ts = _utc_now()
        bridge_revision = req.payload.get("bridge_revision")
        cmd_topic = f"{session_topic_prefix(req.session_id)}entity_pose_cmd"
        err = classify_topic(req.session_id, cmd_topic)
        if err:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code=err,
                error_message="topic not allowed",
            )
        self._state.published.append(
            {
                "topic": cmd_topic,
                "entity_id": entity_id,
                "pose": pose,
            }
        )
        state_topic = f"{session_topic_prefix(req.session_id)}entity_state"
        self._state.published.append(
            {
                "topic": state_topic,
                "payload": self._state.entity_state_payload(),
            }
        )
        result: dict[str, Any] = {
            "entity_id": entity_id,
            "sim_entity_ref": sim_ref,
            "topics": [cmd_topic, state_topic],
            "sync_seq": sync_seq,
            "feedback_timestamp_utc": ts,
        }
        if bridge_revision is not None:
            result["bridge_revision"] = int(bridge_revision)
        if self._state.mode == "live" and self._state.live_ros is not None:
            self._state.live_ros.publish_pose_cmd(
                op="apply",
                entity_id=entity_id,
                entity_type=entity_type,
                pose=pose,
                sim_entity_ref=sim_ref,
                bridge_revision=int(bridge_revision) if bridge_revision is not None else None,
            )
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result=result,
        )

    def _delete_entity(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        entity_id = str(req.payload["entity_id"])
        sim_ref = self._state.sim_entity_refs.get(entity_id)
        self._state.entities.pop(entity_id, None)
        self._state.sim_entity_refs.pop(entity_id, None)
        if self._state.mode == "live" and self._state.live_ros is not None:
            self._state.live_ros.publish_pose_cmd(
                op="delete",
                entity_id=entity_id,
                entity_type="drone",
                pose={},
                sim_entity_ref=sim_ref,
            )
        state_topic = f"{session_topic_prefix(req.session_id)}entity_state"
        self._state.published.append(
            {
                "topic": state_topic,
                "payload": self._state.entity_state_payload(),
            }
        )
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"entity_id": entity_id, "deleted": True},
        )

    def _reset_world(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        self._state.entities.clear()
        self._state.drift_offsets.clear()
        self._state.sync_seq = 0
        self._state.telemetry_seq = 0
        self._state.published.clear()
        if self._state.mode == "live" and self._state.live_ros is not None:
            refs = list(self._state.sim_entity_refs.items())
            for eid, sim_ref in refs:
                self._state.live_ros.publish_pose_cmd(
                    op="delete",
                    entity_id=eid,
                    entity_type="drone",
                    pose={},
                    sim_entity_ref=sim_ref,
                )
        self._state.sim_entity_refs.clear()
        self._record_clock_topic()
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"reset": True},
        )

    def _validate_topic(self, req: IpcRequest) -> IpcResponse:
        topic = str(req.payload.get("topic", ""))
        err = classify_topic(req.session_id, topic)
        if err:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code=err,
                error_message="ros_allowlist_reject",
            )
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"allowed": True, "topic": topic},
        )

    def _mock_publish(self, req: IpcRequest) -> IpcResponse:
        topic = str(req.payload.get("topic", ""))
        err = classify_topic(req.session_id, topic)
        if err:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code=err,
                error_message="ros_allowlist_reject",
            )
        if self._state is not None:
            self._state.published.append(
                {"topic": topic, "payload": req.payload.get("message")}
            )
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"published": True, "topic": topic},
        )

    def _poll_feedback(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        inject = req.payload.get("mock_inject_drift")
        if isinstance(inject, dict) and inject.get("entity_id"):
            eid = str(inject["entity_id"])
            offset = dict(inject.get("offset") or {})
            self._state.drift_offsets[eid] = offset
        live_feedback = None
        if self._state.mode == "live" and self._state.live_ros is not None:
            live_feedback = self._state.live_ros.get_feedback()
        if live_feedback is not None:
            entities = list(live_feedback.get("entities") or [])
            self._state.sync_seq = int(live_feedback.get("sync_seq") or self._state.sync_seq)
            return IpcResponse(
                request_id=req.request_id,
                ok=True,
                result={
                    "schema": "rt_adapter_feedback_v1",
                    "timestamp_utc": live_feedback.get("timestamp_utc") or _utc_now(),
                    "sync_seq": self._state.sync_seq,
                    "entities": entities,
                    "source": "live_gazebo",
                },
            )
        entities = [
            {
                "entity_id": eid,
                "entity_type": ent["entity_type"],
                "pose": self._state.feedback_pose_for(eid),
                "sim_entity_ref": self._state.sim_entity_refs.get(eid),
                **self._state.entity_telemetry_fields_for(eid),
            }
            for eid, ent in self._state.entities.items()
        ]
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={
                "schema": "rt_adapter_feedback_v1",
                "timestamp_utc": _utc_now(),
                "sync_seq": self._state.sync_seq,
                "entities": entities,
            },
        )

    def _mock_inject_drift(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        entity_id = str(req.payload["entity_id"])
        offset = dict(req.payload.get("offset") or {})
        self._state.drift_offsets[entity_id] = offset
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={
                "entity_id": entity_id,
                "offset": offset,
                "feedback_pose": self._state.feedback_pose_for(entity_id),
            },
        )

    def _resync_all(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        entities_in = list(req.payload.get("entities") or [])
        applied = 0
        for item in entities_in:
            eid = str(item.get("entity_id", ""))
            if not eid:
                continue
            entity_type = str(item.get("entity_type", "drone"))
            pose = dict(item.get("pose") or {})
            sim_ref = self._state.sim_entity_refs.get(eid)
            if sim_ref is None:
                sim_ref = f"sim-{uuid.uuid4().hex[:8]}"
                self._state.sim_entity_refs[eid] = sim_ref
            self._state.entities[eid] = default_entity_record(entity_type, pose)
            self._state.drift_offsets.pop(eid, None)
            applied += 1
        self._state.sync_seq += 1
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"applied": applied, "sync_seq": self._state.sync_seq},
        )

    def _poll_telemetry(self, req: IpcRequest) -> IpcResponse:
        if self._state is None:
            return IpcResponse(
                request_id=req.request_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="not attached",
            )
        self._state.telemetry_seq += 1
        mock_stale = bool(req.payload.get("mock_stale_telemetry"))
        ts = "2000-01-01T00:00:00+00:00" if mock_stale else _utc_now()
        alive = True
        if self._state.launch_proc is not None:
            alive = self._state.launch_proc.poll() is None
        if self._state.mode != "live" and self._state.kinematic_plant_enabled and not mock_stale:
            now = time.monotonic()
            last = self._state.last_poll_monotonic
            dt = max(0.0, now - float(last)) if last is not None else 0.0
            self._state.last_poll_monotonic = now
            if dt > 0.0:
                for ent in self._state.entities.values():
                    commanded = dict(ent.get("commanded_pose") or ent.get("pose") or {})
                    advance_entity_toward_command(
                        ent,
                        commanded,
                        limits=self._state.kinematic_limits,
                        plant_enabled=True,
                        aero=self._state.aero,
                        dt=dt,
                        now=now,
                    )
        entities = [
            {
                "entity_id": eid,
                "entity_type": ent["entity_type"],
                "pose": self._state.feedback_pose_for(eid),
                "sim_entity_ref": self._state.sim_entity_refs.get(eid),
                **self._state.entity_telemetry_fields_for(eid),
            }
            for eid, ent in self._state.entities.items()
        ]
        result: dict[str, Any] = {
            "schema": "rt_adapter_telemetry_v1",
            "timestamp_utc": ts,
            "telemetry_seq": self._state.telemetry_seq,
            "clock_mirror": self._state.clock_payload(),
            "adapter_health": {
                "alive": alive,
                "mode": self._state.mode,
                "paused": self._state.paused,
                "entity_count": len(self._state.entities),
            },
            "entity_pose_mirror": {"entities": entities},
            "world_revision_hint": {
                "telemetry_seq": self._state.telemetry_seq,
                "sync_seq": self._state.sync_seq,
            },
        }
        coupling_on = self._state.enable_fidelity_coupling or bool(
            req.payload.get("enable_fidelity_coupling")
        )
        if coupling_on:
            from rt_sandbox.fidelity_coupling import (
                build_entity_truth_rows,
                build_fidelity_truth_snapshot,
            )

            entity_truth = build_entity_truth_rows(
                entities,
                feedback_pose_for=self._state.feedback_pose_for,
                ground_z_m=self._state.fidelity_ground_z_m,
            )
            attestation = "stale" if mock_stale else "available"
            result["fidelity_truth"] = build_fidelity_truth_snapshot(
                session_id=self._state.session_id,
                timestamp_utc=ts,
                entity_truth=entity_truth,
                attestation_status=attestation,
                ground_z_m=self._state.fidelity_ground_z_m,
            )
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result=result,
        )

    def _record_clock_topic(self) -> None:
        if self._state is None:
            return
        topic = f"{session_topic_prefix(self._state.session_id)}clock"
        if classify_topic(self._state.session_id, topic):
            return
        self._state.published.append(
            {"topic": topic, "payload": self._state.clock_payload()}
        )


def main() -> None:
    AdapterWorker().run_loop()


if __name__ == "__main__":
    main()
