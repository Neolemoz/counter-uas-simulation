"""Per-session state bag for RT sandbox bridge (PLAT-RT-R3a)."""

from __future__ import annotations

from dataclasses import dataclass, field

from rt_sandbox.lifecycle import SessionState
from rt_sandbox.pose_sync import PoseSyncMirror
from rt_sandbox.runtime_handle import RuntimeHandle
from rt_sandbox.runtime_stub import RuntimeStub
from rt_sandbox.telemetry_bridge import TelemetryMirror
from rt_sandbox.runtime_capture import RuntimeCaptureState
from rt_sandbox.tactical_controller import TacticalController
from rt_sandbox.workflow import WorkflowState
from rt_sandbox.world_state import WorldStateStore


@dataclass
class SessionRecord:
    session_id: str
    state: SessionState
    created_monotonic: float
    bridge_ready_deadline: float
    cleanup_after: float | None = None
    runtime: RuntimeHandle = field(default_factory=RuntimeStub)
    world: WorldStateStore | None = None
    pose_sync: PoseSyncMirror | None = None
    telemetry_mirror: TelemetryMirror | None = None
    issued_by: str = "rt_ui_prototype"
    workflow: WorkflowState | None = None
    template_apply_count: int = 0
    templates_applied: list[str] = field(default_factory=list)
    tactical: TacticalController | None = None
    live_assignments: dict[str, str] = field(default_factory=dict)
    runtime_capture: RuntimeCaptureState | None = None
    runtime_profile: str = "stub"
    last_live_background_poll_monotonic: float | None = None
    last_live_background_poll_utc: str | None = None
