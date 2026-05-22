"""Lightweight runtime child process stub (no Gazebo/ROS)."""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from typing import Any


@dataclass
class RuntimeStub:
    """Simulated runtime child for orphan/cleanup tests."""

    kind: str = "stub"
    pid: int | None = None
    paused: bool = False
    _proc: subprocess.Popen[bytes] | None = None

    def start(self) -> int:
        if self._proc is not None and self._proc.poll() is None:
            return self._proc.pid
        self._proc = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(86400)"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.pid = self._proc.pid
        self.paused = False
        return self.pid

    def is_alive(self) -> bool:
        if self._proc is None:
            return False
        return self._proc.poll() is None

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def stop(self) -> None:
        self.paused = True

    def terminate(self) -> None:
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

    def health_payload(self) -> dict[str, Any]:
        return {
            "stub_alive": self.is_alive(),
            "adapter_alive": False,
            "adapter_mode": None,
            "adapter_pid": None,
        }

    def apply_pose(
        self,
        entity_id: str,
        entity_type: str,
        pose: dict[str, float],
    ) -> dict[str, Any] | None:
        return None

    def delete_entity(self, entity_id: str) -> dict[str, Any] | None:
        return None

    def reset_world(self) -> None:
        return
