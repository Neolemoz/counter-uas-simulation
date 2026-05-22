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
            line = ""
            while time.monotonic() < deadline:
                if self._proc.stdout is None:
                    break
                line = self._proc.stdout.readline()
                if line:
                    break
                if self._proc.poll() is not None:
                    break
                time.sleep(0.01)
        if not line:
            return IpcResponse(
                request_id=req_id,
                ok=False,
                error_code="RUNTIME_UNAVAILABLE",
                error_message="adapter_ipc_timeout",
            )
        return IpcResponse.from_line(line)

    def start(self) -> int:
        if self._attached and self.pid is not None:
            return self.pid
        self._ensure_worker()
        payload: dict[str, Any] = {"mode": self.mode}
        if self.ros_domain_id is not None:
            payload["ros_domain_id"] = self.ros_domain_id
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
    ) -> dict[str, Any] | None:
        resp = self._request(
            "apply_pose",
            {
                "entity_id": entity_id,
                "entity_type": entity_type,
                "pose": pose,
            },
        )
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
