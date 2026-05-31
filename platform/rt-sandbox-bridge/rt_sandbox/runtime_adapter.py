"""Gazebo/ROS runtime adapter — subprocess IPC to adapter_worker (PLAT-RT-G2)."""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rt_sandbox.adapter_ipc import IpcRequest, IpcResponse


def _bridge_pkg_root() -> Path:
    return Path(__file__).resolve().parent.parent


@dataclass
class GazeboRuntimeAdapter:
    session_id: str
    mode: str = "mock"
    ipc_timeout_s: float = 5.0
    ready_timeout_s: float = 60.0
    ros_domain_id: int | None = None
    rt_sandbox_world: str = "rt_sandbox_flat"
    ground_snap_enabled: bool = True
    enable_fidelity_coupling: bool = False
    fidelity_ground_z_m: float = 0.0

    kind: str = "adapter"
    pid: int | None = None
    paused: bool = False
    _proc: subprocess.Popen[str] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _attached: bool = False
    _last_sync: dict[str, Any] | None = None

    def _ensure_worker(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        pkg = _bridge_pkg_root()
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(
            [str(pkg), env.get("PYTHONPATH", "")]
        ).strip(os.pathsep)
        self._proc = subprocess.Popen(
            [sys.executable, "-m", "rt_sandbox.adapter_worker"],
            cwd=str(pkg),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            env=env,
        )
        self.pid = self._proc.pid

    def _request(self, op: str, payload: dict[str, Any] | None = None) -> IpcResponse:
        self._ensure_worker()
        assert self._proc is not None and self._proc.stdin is not None
        req_id = str(uuid.uuid4())
        req = IpcRequest(
            op=op,
            request_id=req_id,
            session_id=self.session_id,
            payload=payload or {},
        )
        with self._lock:
            self._proc.stdin.write(req.to_line())
            self._proc.stdin.flush()
            deadline = time.monotonic() + self.ipc_timeout_s
            while time.monotonic() < deadline:
                if self._proc.stdout is None:
                    break
                line = self._proc.stdout.readline()
                if not line:
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.01)
                    continue
                try:
                    resp = IpcResponse.from_line(line)
                except ValueError:
                    continue
                if resp.request_id != req_id:
                    continue
                return resp
        return IpcResponse(
            request_id=req_id,
            ok=False,
            error_code="RUNTIME_UNAVAILABLE",
            error_message="adapter_ipc_timeout",
        )

    def start(self) -> int:
        if self._attached and self.pid is not None:
            return self.pid
        self._ensure_worker()
        payload: dict[str, Any] = {"mode": self.mode}
        if self.ros_domain_id is not None:
            payload["ros_domain_id"] = self.ros_domain_id
        payload["rt_sandbox_world"] = self.rt_sandbox_world
        payload["ground_snap_enabled"] = self.ground_snap_enabled
        payload["enable_fidelity_coupling"] = self.enable_fidelity_coupling
        payload["fidelity_ground_z_m"] = self.fidelity_ground_z_m
        payload["ready_timeout_s"] = self.ready_timeout_s
        resp = self._request("attach", payload)
        if not resp.ok:
            raise OSError(resp.error_message or resp.error_code or "attach failed")
        self._attached = True
        self.paused = False
        worker_pid = int(resp.result.get("worker_pid") or self.pid or 0)
        self.pid = worker_pid
        return worker_pid

    def is_alive(self) -> bool:
        if self._proc is None:
            return False
        if self._proc.poll() is not None:
            return False
        return self._attached

    def pause(self) -> None:
        resp = self._request("pause")
        if resp.ok:
            self.paused = True

    def resume(self) -> None:
        resp = self._request("resume")
        if resp.ok:
            self.paused = False

    def stop(self) -> None:
        self.paused = True

    def terminate(self) -> None:
        if self._attached:
            self._request("detach")
            self._attached = False
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait(timeout=2)
        self._proc = None
        self.pid = None
        self.paused = False

    def kill_for_crash_simulation(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.kill()
            self._proc.wait(timeout=2)
        self._proc = None
        self.pid = None
        self._attached = False

    def health_payload(self) -> dict[str, Any]:
        alive = self.is_alive() if self._attached else False
        return {
            "stub_alive": False,
            "adapter_alive": alive,
            "adapter_mode": self.mode,
            "adapter_pid": self.pid,
        }

    def apply_pose(
        self,
        entity_id: str,
        entity_type: str,
        pose: dict[str, float],
        *,
        bridge_revision: int | None = None,
    ) -> dict[str, Any] | None:
        payload: dict[str, Any] = {
            "entity_id": entity_id,
            "entity_type": entity_type,
            "pose": pose,
        }
        if bridge_revision is not None:
            payload["bridge_revision"] = bridge_revision
        resp = self._request("apply_pose", payload)
        if not resp.ok:
            return {"error_code": resp.error_code, "error_message": resp.error_message}
        self._last_sync = dict(resp.result)
        return dict(resp.result)

    def delete_entity(self, entity_id: str) -> dict[str, Any] | None:
        resp = self._request("delete_entity", {"entity_id": entity_id})
        if not resp.ok:
            return {"error_code": resp.error_code, "error_message": resp.error_message}
        return dict(resp.result)

    def reset_world(self) -> None:
        self._request("reset_world")

    def validate_topic(self, topic: str) -> IpcResponse:
        return self._request("validate_topic", {"topic": topic})

    def last_sync(self) -> dict[str, Any] | None:
        return self._last_sync

    def poll_feedback(
        self,
        *,
        mock_inject_drift: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        payload: dict[str, Any] = {}
        if mock_inject_drift:
            payload["mock_inject_drift"] = mock_inject_drift
        resp = self._request("poll_feedback", payload)
        if not resp.ok:
            return {
                "error_code": resp.error_code or "ADAPTER_FEEDBACK_LOST",
                "error_message": resp.error_message,
            }
        return dict(resp.result)

    def mock_inject_drift(
        self,
        entity_id: str,
        offset: dict[str, float],
    ) -> dict[str, Any] | None:
        resp = self._request(
            "mock_inject_drift",
            {"entity_id": entity_id, "offset": offset},
        )
        if not resp.ok:
            return {"error_code": resp.error_code, "error_message": resp.error_message}
        return dict(resp.result)

    def resync_all(self, entities: list[dict[str, Any]]) -> dict[str, Any] | None:
        resp = self._request("resync_all", {"entities": entities})
        if not resp.ok:
            return {"error_code": resp.error_code, "error_message": resp.error_message}
        return dict(resp.result)

    def poll_telemetry(
        self,
        *,
        mock_stale_telemetry: bool = False,
        enable_fidelity_coupling: bool = False,
    ) -> dict[str, Any] | None:
        payload: dict[str, Any] = {}
        if mock_stale_telemetry:
            payload["mock_stale_telemetry"] = True
        if enable_fidelity_coupling:
            payload["enable_fidelity_coupling"] = True
            payload["fidelity_ground_z_m"] = self.fidelity_ground_z_m
        resp = self._request("poll_telemetry", payload)
        if not resp.ok:
            return {
                "error_code": resp.error_code or "ADAPTER_FEEDBACK_LOST",
                "error_message": resp.error_message,
            }
        return dict(resp.result)
