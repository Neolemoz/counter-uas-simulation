"""Deny-by-default command governance (rt_bridge_contract_v1, rt_runtime_governance_v1).

Runtime subcommand allow-list: see docs/evaluation/rt_runtime_subcommand_registry_v1.md.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

GOVERNANCE_BANNER = "RT SANDBOX — experimental simulation; not operational state"

SESSION_COMMANDS = frozenset(
    {
        "start_session",
        "pause_session",
        "resume",
        "stop_session",
        "discard_session",
        "reset_session",
        "capture_session",
    }
)

SIM_COMMAND_ALIASES = frozenset(
    {
        "start_sim",
        "pause_sim",
        "resume_sim",
        "stop_sim",
        "reset_sim",
        "spawn_attacker",
        "spawn_defender",
        "apply_scenario",
        "assign_target",
        "cancel_assignment",
        "reposition_entity",
        "start_capture",
        "stop_capture",
        "capture_status",
    }
)

REGISTRY_COMMANDS = frozenset(
    {
        "list_sessions",
        "set_editing_session",
    }
)

HANDOFF_READ_COMMANDS = frozenset(
    {
        "list_capture_handoff_status",
    }
)

ENTITY_COMMANDS = frozenset(
    {
        "spawn_entity",
        "move_entity",
        "delete_entity",
    }
)

TELEMETRY_COMMANDS = frozenset(
    {
        "subscribe_telemetry",
        "unsubscribe_telemetry",
    }
)

TEMPLATE_COMMANDS = frozenset(
    {
        "list_runtime_templates",
        "apply_runtime_template",
    }
)

WORKFLOW_COMMANDS = frozenset(
    {
        "start_workflow",
        "advance_workflow",
        "reset_workflow",
        "reload_workflow",
        "get_workflow_state",
    }
)

RUNTIME_COMMANDS = frozenset({"send_runtime_command"})

TACTICAL_COMMANDS = frozenset(
    {
        "set_tactical_mode",
        "select_candidate",
        "assign_candidate",
        "clear_assignment",
        "get_tactical_state",
        "request_recommendation",
        "approve_recommendation",
        "reject_recommendation",
        "pause_autonomous_loop",
        "resume_autonomous_loop",
    }
)

ALLOWED_COMMANDS = (
    SESSION_COMMANDS
    | SIM_COMMAND_ALIASES
    | REGISTRY_COMMANDS
    | HANDOFF_READ_COMMANDS
    | ENTITY_COMMANDS
    | TELEMETRY_COMMANDS
    | TEMPLATE_COMMANDS
    | WORKFLOW_COMMANDS
    | RUNTIME_COMMANDS
    | TACTICAL_COMMANDS
)

ENTITY_CATALOG = frozenset({"radar", "interceptor", "drone", "waypoint_marker"})

ENTITY_TYPE_LIMITS: dict[str, int] = {
    "radar": 8,
    "interceptor": 8,
    "drone": 8,
    "waypoint_marker": 8,
}

MAX_ENTITY_COUNT = 32

WORLD_BOUNDS: dict[str, dict[str, float]] = {
    "x": {"min": -7000.0, "max": 7000.0},
    "y": {"min": -7000.0, "max": 7000.0},
    "z": {"min": 0.0, "max": 200.0},
}

RUNTIME_SUBCOMMANDS = frozenset(
    {
        "adapter_attach",
        "adapter_detach",
        "adapter_health",
        "adapter_poll_feedback",
        "adapter_poll_telemetry",
        "adapter_resync",
        "mock_inject_drift",
    }
)

RUNTIME_SUBCOMMANDS_RESERVED = frozenset(
    {
        "reload_world_config",
        "set_clock_pause",
    }
)

RUNTIME_SUBCOMMAND_AUDIT_EXCEPTIONS: dict[str, str] = {
    "adapter_resync": "sync_update",
}

SESSION_RUNTIME_PROFILES = frozenset({"stub", "mock_adapter"})

RT_FORBIDDEN_COMMANDS = frozenset(
    {
        "engage",
        "intercept",
        "strike",
        "corpus_promote",
        "federation_register",
        "publish_to_collection",
        "launch_queue",
        "run_experiment_queue",
        "publish_topic",
        "alter_parser_contract",
        "import_scenario",
        "import_replay",
        "auto_capture",
        "publish_template",
        "save_template_to_corpus",
        "promote_workflow",
        "orchestration_apply",
    }
)

FORBIDDEN_SUBSTRINGS = (
    "corpus_",
    "federation_",
    "orchestration",
    "replay_sa",
    "publish_topic",
)


@dataclass
class GovernanceConfig:
    max_concurrent_sessions: int = 3
    max_total_entities_across_sessions: int = 64
    background_telemetry_pull_cap_hz: float = 1.0
    command_rate_burst: int = 5
    command_rate_sustained: float = 1.0
    bridge_ready_timeout_s: float = 60.0
    session_cleanup_timeout_s: float = 120.0
    cleanup_pending_max_age_s: float = 300.0
    max_session_duration_s: float = 3600.0
    max_entity_count: int = MAX_ENTITY_COUNT
    telemetry_update_rate_cap_hz: float = 10.0
    max_telemetry_channels_per_subscription: int = 5
    telemetry_ring_buffer_size: int = 64
    max_capture_bundle_bytes: int = 5 * 1024 * 1024
    max_staged_captures: int = 32
    max_runtime_templates_in_catalog: int = 16
    max_entities_per_template_apply: int = 8
    max_template_applies_per_session: int = 32
    max_workflows_in_catalog: int = 8
    max_workflow_steps: int = 12
    authority_scope: str = "rt_sandbox_prototype"
    enable_gazebo_adapter: bool = False
    adapter_mode: str = "mock"
    adapter_ready_timeout_s: float = 60.0
    adapter_ipc_timeout_s: float = 5.0
    ros_domain_id_offset: int = 42
    pose_sync_enabled: bool = True
    pose_sync_drift_threshold_m: float = 2.0
    adapter_feedback_stale_s: float = 30.0
    telemetry_bridge_enabled: bool = True
    telemetry_stale_s: float = 30.0
    capture_normalization_enabled: bool = True
    rt_sandbox_world: str = "rt_sandbox_flat"
    pose_sync_yaw_threshold_deg: float = 0.0
    adapter_live_background_poll_hz: float = 0.0
    entity_ground_snap_enabled: bool = True
    enable_fidelity_coupling: bool = False
    fidelity_truth_stale_s: float = 30.0
    fidelity_ground_z_m: float = 0.0

    def ros_domain_id_for_session(self, session_id: str) -> int:
        """Derive isolated ROS_DOMAIN_ID from session UUID (live mode only)."""
        try:
            tail = int(session_id.replace("-", "")[:8], 16)
        except ValueError:
            tail = 0
        return (self.ros_domain_id_offset + (tail % 100)) % 232


@dataclass
class RateLimiter:
    burst: int
    sustained_per_s: float
    _timestamps: deque[float] = field(default_factory=deque)

    def check(self, now: float | None = None) -> bool:
        t = now if now is not None else time.monotonic()
        window = 1.0
        while self._timestamps and t - self._timestamps[0] > window:
            self._timestamps.popleft()
        if len(self._timestamps) >= self.burst:
            return False
        if len(self._timestamps) >= self.burst - 1 and self._timestamps:
            elapsed = t - self._timestamps[-1]
            if elapsed < 1.0 / self.sustained_per_s:
                return False
        self._timestamps.append(t)
        return True


def classify_command(command_type: str) -> str | None:
    """Return error_code if forbidden, else None."""
    if command_type in RT_FORBIDDEN_COMMANDS:
        return "COMMAND_FORBIDDEN"
    low = command_type.lower()
    for sub in FORBIDDEN_SUBSTRINGS:
        if sub in low:
            return "COMMAND_FORBIDDEN"
    if command_type == "resume_session":
        return "COMMAND_FORBIDDEN"
    if command_type not in ALLOWED_COMMANDS:
        return "COMMAND_FORBIDDEN"
    return None


def validate_start_session_payload(payload: Any) -> str | None:
    """Optional additive start_session payload — stub default; mock_adapter only."""
    if payload is None:
        return None
    if not isinstance(payload, dict):
        return "COMMAND_FORBIDDEN"
    if not payload:
        return None
    extra = set(payload.keys()) - {"runtime_profile"}
    if extra:
        return "COMMAND_FORBIDDEN"
    profile = payload.get("runtime_profile", "stub")
    if not isinstance(profile, str) or profile not in SESSION_RUNTIME_PROFILES:
        return "COMMAND_FORBIDDEN"
    return None


def validate_capture_payload(command_type: str, payload: Any) -> str | None:
    """Return error_code if capture payload invalid."""
    if command_type != "capture_session":
        return None
    from rt_sandbox.capture import validate_capture_payload as _validate

    return _validate(payload)


def pose_in_bounds(pose: dict[str, float]) -> bool:
    for axis in ("x", "y", "z"):
        if axis not in pose:
            return False
        bounds = WORLD_BOUNDS[axis]
        v = float(pose[axis])
        if v < bounds["min"] or v > bounds["max"]:
            return False
    return True


def validate_pose(pose: Any) -> str | None:
    """Return error_code if invalid."""
    if not isinstance(pose, dict):
        return "INVALID_POSE"
    for key in ("x", "y", "z"):
        if key not in pose:
            return "INVALID_POSE"
        try:
            float(pose[key])
        except (TypeError, ValueError):
            return "INVALID_POSE"
    if "yaw_deg" in pose:
        try:
            float(pose["yaw_deg"])
        except (TypeError, ValueError):
            return "INVALID_POSE"
    if not pose_in_bounds({k: float(pose[k]) for k in ("x", "y", "z")}):
        return "INVALID_POSE"
    return None


def validate_entity_payload(command_type: str, payload: Any) -> str | None:
    """Return error_code if payload invalid."""
    if not isinstance(payload, dict):
        return "INVALID_POSE"
    if command_type == "spawn_entity":
        entity_type = payload.get("entity_type")
        if not isinstance(entity_type, str) or entity_type not in ENTITY_CATALOG:
            return "COMMAND_FORBIDDEN"
        pose = payload.get("pose")
        return validate_pose(pose)
    if command_type == "move_entity":
        entity_id = payload.get("entity_id")
        if not isinstance(entity_id, str) or not entity_id:
            return "ENTITY_NOT_FOUND"
        return validate_pose(payload.get("pose"))
    if command_type == "delete_entity":
        entity_id = payload.get("entity_id")
        if not isinstance(entity_id, str) or not entity_id:
            return "ENTITY_NOT_FOUND"
    return None


def validate_telemetry_payload(command_type: str, payload: Any) -> str | None:
    """Return error_code if payload invalid."""
    from rt_sandbox.telemetry_subscriptions import (
        MAX_CHANNELS_PER_SUBSCRIPTION,
        TELEMETRY_CHANNELS,
    )

    if not isinstance(payload, dict):
        return "COMMAND_FORBIDDEN"
    if command_type == "subscribe_telemetry":
        channels = payload.get("channels")
        if not isinstance(channels, list) or not channels:
            return "COMMAND_FORBIDDEN"
        if len(channels) > MAX_CHANNELS_PER_SUBSCRIPTION:
            return "RESOURCE_LIMIT_EXCEEDED"
        for ch in channels:
            if not isinstance(ch, str) or ch not in TELEMETRY_CHANNELS:
                return "COMMAND_FORBIDDEN"
        return None
    if command_type == "unsubscribe_telemetry":
        sub_id = payload.get("subscription_id")
        if not isinstance(sub_id, str) or not sub_id:
            return "SESSION_NOT_FOUND"
    return None


def validate_template_command_payload(command_type: str, payload: Any) -> str | None:
    if command_type == "list_runtime_templates":
        return None
    if command_type == "apply_runtime_template":
        from rt_sandbox.templates import validate_template_payload

        return validate_template_payload(payload)
    return None


def validate_workflow_command_payload(command_type: str, payload: Any) -> str | None:
    if command_type == "get_workflow_state":
        return None
    if command_type in {"reset_workflow"}:
        return None
    if command_type in {"start_workflow", "reload_workflow"}:
        from rt_sandbox.workflow import validate_workflow_payload

        return validate_workflow_payload(payload)
    if command_type == "advance_workflow":
        return None
    return None


def validate_runtime_subcommand(payload: Any) -> str | None:
    """Return error_code if send_runtime_command payload invalid."""
    if not isinstance(payload, dict):
        return "COMMAND_FORBIDDEN"
    sub = payload.get("sub_command")
    if not isinstance(sub, str) or sub not in RUNTIME_SUBCOMMANDS:
        return "COMMAND_FORBIDDEN"
    return None
