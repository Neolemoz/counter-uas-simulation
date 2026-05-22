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
from pathlib import Path
from typing import Any

from rt_sandbox.adapter_ipc import IpcRequest, IpcResponse
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
    gazebo_pid: int | None = None
    launch_proc: subprocess.Popen[bytes] | None = None

    def clock_payload(self) -> dict[str, Any]:
        return {"paused": self.paused, "mode": self.mode}

    def entity_state_payload(self) -> dict[str, Any]:
        return {
            "entities": [
                {
                    "entity_id": eid,
                    "entity_type": ent["entity_type"],
                    "pose": dict(ent["pose"]),
                    "sim_entity_ref": self.sim_entity_refs.get(eid),
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
            "gazebo_target_sim",
            "gazebo_target.launch.py",
            "use_gazebo_gui:=false",
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
        except OSError as exc:
            return None, str(exc)
        assert self._state is not None
        self._state.launch_proc = proc
        time.sleep(0.5)
        if proc.poll() is not None:
            return None, "gazebo launch exited immediately"
        return proc.pid, None

    def _detach(self, req: IpcRequest) -> IpcResponse:
        self._teardown_launch()
        self._state = None
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={"detached": True},
        )

    def _teardown_launch(self) -> None:
        if self._state is None:
            return
        proc = self._state.launch_proc
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
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
        self._state.entities[entity_id] = {
            "entity_type": entity_type,
            "pose": pose,
        }
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
        return IpcResponse(
            request_id=req.request_id,
            ok=True,
            result={
                "entity_id": entity_id,
                "sim_entity_ref": sim_ref,
                "topics": [cmd_topic, state_topic],
            },
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
        self._state.entities.pop(entity_id, None)
        self._state.sim_entity_refs.pop(entity_id, None)
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
        self._state.sim_entity_refs.clear()
        self._state.published.clear()
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
