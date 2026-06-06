"""Unit tests for RT sandbox Gazebo WorldControl helpers (PLAT-RT-LIVE-CONTROL-TRUTH1)."""

from __future__ import annotations

import sys
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_GZ_PKG = _REPO / "src" / "rt_sandbox_gz"
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
for path in (_GZ_PKG, _BRIDGE_PKG):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rt_sandbox.adapter_ipc import IpcRequest  # noqa: E402
from rt_sandbox.adapter_worker import AdapterWorker, MockSimState  # noqa: E402
from rt_sandbox_gz import gz_tools  # noqa: E402


def test_world_control_fmt_strings() -> None:
    assert gz_tools.fmt_world_control_pause() == "pause: true"
    assert gz_tools.fmt_world_control_resume() == "pause: false"
    assert gz_tools.fmt_world_control_reset_all() == "reset: {all: true}"


def test_world_control_reqtype_for_control_service() -> None:
    assert gz_tools._reqtype_for("/world/rt_sandbox_flat/control") == "gz.msgs.WorldControl"


def test_gz_world_pause_resume_reset_all(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str, str]] = []

    def _fake_gz_service(world_name: str, service: str, req: str, timeout_ms: int = 3000) -> bool:
        calls.append((world_name, service, req))
        return True

    monkeypatch.setattr(gz_tools, "gz_service", _fake_gz_service)
    assert gz_tools.gz_world_pause("rt_sandbox_flat") is True
    assert gz_tools.gz_world_resume("demo_world") is True
    assert gz_tools.gz_world_reset_all("demo_world") is True
    assert calls == [
        ("rt_sandbox_flat", gz_tools.WORLD_CONTROL_SERVICE, "pause: true"),
        ("demo_world", gz_tools.WORLD_CONTROL_SERVICE, "pause: false"),
        ("demo_world", gz_tools.WORLD_CONTROL_SERVICE, "reset: {all: true}"),
    ]


def test_adapter_worker_live_pause_resume_invoke_gz(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    worker._state = MockSimState(
        session_id=str(uuid.uuid4()),
        mode="live",
        world_name="rt_sandbox_flat",
    )
    calls: list[str] = []

    class _Mod:
        @staticmethod
        def gz_world_pause(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append(f"pause:{world_name}")
            return True

        @staticmethod
        def gz_world_resume(world_name: str, timeout_ms: int = 3000) -> bool:
            calls.append(f"resume:{world_name}")
            return True

    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: "/usr/bin/gz")
    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: _Mod())

    req_id = str(uuid.uuid4())
    pause = worker._pause(IpcRequest(op="pause", request_id=req_id, session_id=worker._state.session_id))
    assert pause.ok is True
    assert pause.result == {"paused": True}
    assert worker._state.paused is True

    resume = worker._resume(
        IpcRequest(op="resume", request_id=str(uuid.uuid4()), session_id=worker._state.session_id)
    )
    assert resume.ok is True
    assert resume.result == {"paused": False}
    assert worker._state.paused is False
    assert calls == ["pause:rt_sandbox_flat", "resume:rt_sandbox_flat"]


def test_adapter_worker_mock_pause_skips_gz(monkeypatch: pytest.MonkeyPatch) -> None:
    worker = AdapterWorker()
    worker._state = MockSimState(session_id=str(uuid.uuid4()), mode="mock")
    called = {"n": 0}

    class _Mod:
        @staticmethod
        def gz_world_pause(world_name: str, timeout_ms: int = 3000) -> bool:
            called["n"] += 1
            return True

    monkeypatch.setattr("rt_sandbox.adapter_worker._gz_tools_module", lambda: _Mod())
    monkeypatch.setattr("rt_sandbox.adapter_worker.shutil.which", lambda _: "/usr/bin/gz")
    out = worker._pause(
        IpcRequest(op="pause", request_id=str(uuid.uuid4()), session_id=worker._state.session_id)
    )
    assert out.ok is True
    assert called["n"] == 0
