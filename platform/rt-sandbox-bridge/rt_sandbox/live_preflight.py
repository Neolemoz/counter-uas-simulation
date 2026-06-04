"""Live Gazebo runtime preflight checks (Web ↔ Gazebo Step 2).

Read-only host probes — bridge only; no auto-install.
"""

from __future__ import annotations

import shutil
import subprocess
from typing import Any


def _ros_pkg_available(package: str) -> bool:
    if shutil.which("ros2") is None:
        return False
    try:
        proc = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def check_live_runtime_preflight() -> dict[str, Any]:
    """Return schema rt_live_runtime_preflight_v1 — fail-closed when any check fails."""
    ros2_available = shutil.which("ros2") is not None
    gz_available = shutil.which("gz") is not None
    rt_sandbox_gz_available = _ros_pkg_available("rt_sandbox_gz")

    checks = {
        "ros2_available": ros2_available,
        "gz_available": gz_available,
        "rt_sandbox_gz_available": rt_sandbox_gz_available,
    }
    blockers: list[str] = []
    if not ros2_available:
        blockers.append("ros2 not found on PATH")
    if not gz_available:
        blockers.append("gz (Gazebo Sim) not found on PATH")
    if not rt_sandbox_gz_available:
        blockers.append("rt_sandbox_gz ROS package unavailable (source install/setup.bash)")

    ok = not blockers
    message = "live runtime ready" if ok else "; ".join(blockers)
    return {
        "schema": "rt_live_runtime_preflight_v1",
        "ok": ok,
        "checks": checks,
        "blockers": blockers,
        "message": message,
    }
