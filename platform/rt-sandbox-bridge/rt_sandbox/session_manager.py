"""Transient session manager for RT sandbox bridge prototype (PLAT-RT-R3a facade, PLAT-RT-M2 registry)."""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.governance import (
    ENTITY_COMMANDS,
    ENTITY_TYPE_LIMITS,
    GOVERNANCE_BANNER,
    GovernanceConfig,
    LIVE_ADAPTER_BACKGROUND_POLL_HZ,
    RateLimiter,
    TACTICAL_COMMANDS,
    TEMPLATE_COMMANDS,
    TELEMETRY_COMMANDS,
    WORKFLOW_COMMANDS,
    classify_command,
    validate_capture_payload,
    validate_pose,
    validate_start_session_payload,
)
from rt_sandbox.adapter_poll import run_adapter_poll_tick
from rt_sandbox.live_preflight import check_live_runtime_preflight
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.session_adapter_results import apply_adapter_poll_result
from rt_sandbox.isolation import repo_root_from
from rt_sandbox.lifecycle import SessionState, can_transition
from rt_sandbox.session_capture_handler import capture_session as handle_capture_session
from rt_sandbox.session_entity_handlers import handle_entity
from rt_sandbox.session_lifecycle_handlers import (
    discard_session,
    pause_session,
    reset_session,
    resume_session,
    runtime_crashed as lifecycle_runtime_crashed,
    start_session,
    stop_session,
    tick_timeouts,
)
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_registry import SessionRegistry, TERMINAL_STATES
from rt_sandbox.session_handoff_handlers import (
    handle_list_capture_handoff_status,
)
from rt_sandbox.session_registry_handlers import (
    list_sessions as handle_list_sessions,
    pick_editing_session_after_evict,
    set_editing_session as handle_set_editing_session,
)
from rt_sandbox.runtime_capture import (
    begin_runtime_capture,
    finalize_runtime_capture,
    runtime_capture_status,
    validate_runtime_capture_artifact,
)
from rt_sandbox.session_response import fail, ok, response_base
from rt_sandbox.session_runtime_commands import handle_runtime_command
from rt_sandbox.session_tactical_handlers import (
    handle_tactical,
    tick_tactical_autonomous_for_session,
)
from rt_sandbox.tactical_autonomous import monotonic_now
from rt_sandbox.session_telemetry_coordinator import (
    handle_telemetry,
    poll_telemetry_bridge,
    publish_channels_for_transition,
    publish_telemetry,
)
from rt_sandbox.session_workflow_handlers import (
    handle_template,
    handle_workflow,
    list_runtime_templates,
)
from rt_sandbox.telemetry import TelemetryBuffer
from rt_sandbox.telemetry_subscriptions import TelemetrySubscriptionStore

_SIM_COMMAND_CANONICAL = {
    "start_sim": "start_session",
    "pause_sim": "pause_session",
    "resume_sim": "resume",
    "stop_sim": "stop_session",
    "reset_sim": "reset_session",
    "spawn_attacker": "spawn_entity",
    "spawn_defender": "spawn_entity",
    "reposition_entity": "move_entity",
}

_SIM_ENTITY_ALIASES: dict[str, tuple[str, dict[str, float]]] = {
    "spawn_attacker": (
        "drone",
        {"x": 0.0, "y": 0.0, "z": 20.0, "yaw_deg": 0.0},
    ),
    "spawn_defender": (
        "interceptor",
        {"x": 0.0, "y": 0.0, "z": 10.0, "yaw_deg": 0.0},
    ),
}


def _payload_for_sim_alias(command_type: str, payload: Any) -> Any:
    alias = _SIM_ENTITY_ALIASES.get(command_type)
    if alias is None:
        return payload
    entity_type, default_pose = alias
    if payload is None:
        payload_in: dict[str, Any] = {}
    elif isinstance(payload, dict):
        payload_in = dict(payload)
    else:
        return payload
    pose = payload_in.get("pose") or dict(default_pose)
    out: dict[str, Any] = {"entity_type": entity_type, "pose": pose}
    if payload_in.get("entity_id"):
        out["entity_id"] = payload_in["entity_id"]
    return out


_SCENARIO_TERRAIN_PRESETS = frozenset({"rt_sandbox_flat"})
_SCENARIO_GROUP_TYPES = {
    "assets": {"waypoint_marker", "radar"},
    "defenders": {"interceptor"},
    "attackers": {"drone"},
}
_SCENARIO_DEFAULT_TYPES = {
    "assets": "waypoint_marker",
    "defenders": "interceptor",
    "attackers": "drone",
}


def _validate_scenario_entry(group: str, entry: Any) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(entry, dict):
        return None, "INVALID_POSE"
    entity_type = str(entry.get("entity_type") or _SCENARIO_DEFAULT_TYPES[group])
    if entity_type not in _SCENARIO_GROUP_TYPES[group]:
        return None, "COMMAND_FORBIDDEN"
    pose = entry.get("pose")
    pose_err = validate_pose(pose)
    if pose_err:
        return None, pose_err
    assert isinstance(pose, dict)
    normalized: dict[str, Any] = {
        "entity_type": entity_type,
        "pose": {k: float(pose[k]) for k in ("x", "y", "z")},
    }
    if "yaw_deg" in pose:
        normalized["pose"]["yaw_deg"] = float(pose["yaw_deg"])
    if entry.get("entity_id"):
        normalized["entity_id"] = str(entry["entity_id"])
    return normalized, None


def _validate_scenario_payload(
    payload: Any,
    *,
    config: GovernanceConfig,
) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(payload, dict):
        return None, "INVALID_STATE"
    terrain_preset = payload.get("terrain_preset")
    if not isinstance(terrain_preset, str) or not terrain_preset:
        return None, "INVALID_STATE"
    if terrain_preset not in _SCENARIO_TERRAIN_PRESETS:
        return None, "COMMAND_FORBIDDEN"
    if terrain_preset != config.rt_sandbox_world:
        return None, "COMMAND_FORBIDDEN"

    normalized: dict[str, Any] = {"terrain_preset": terrain_preset}
    type_counts: dict[str, int] = {}
    entity_ids: set[str] = set()
    total = 0
    for group in ("assets", "defenders", "attackers"):
        items = payload.get(group)
        if not isinstance(items, list):
            return None, "INVALID_STATE"
        normalized_items: list[dict[str, Any]] = []
        for item in items:
            normalized_item, err = _validate_scenario_entry(group, item)
            if err:
                return None, err
            assert normalized_item is not None
            normalized_items.append(normalized_item)
            explicit_entity_id = normalized_item.get("entity_id")
            if explicit_entity_id:
                entity_id = str(explicit_entity_id)
                if entity_id in entity_ids:
                    return None, "INVALID_STATE"
                entity_ids.add(entity_id)
            entity_type = str(normalized_item["entity_type"])
            type_counts[entity_type] = type_counts.get(entity_type, 0) + 1
            total += 1
        normalized[group] = normalized_items

    if total > config.max_entity_count:
        return None, "RESOURCE_LIMIT_EXCEEDED"
    for entity_type, count in type_counts.items():
        if count > ENTITY_TYPE_LIMITS.get(entity_type, 0):
            return None, "RESOURCE_LIMIT_EXCEEDED"
    normalized["total_entity_count"] = total
    return normalized, None


def _runtime_profile_from_config(config: GovernanceConfig) -> str:
    if not config.enable_gazebo_adapter:
        return "stub"
    if config.adapter_mode == "live":
        return "live"
    return "mock_adapter"


def _config_for_command(
    config: GovernanceConfig,
    command_type: str,
    payload: Any = None,
) -> GovernanceConfig:
    if command_type == "start_sim":
        return replace(
            config,
            enable_gazebo_adapter=True,
            adapter_mode="live",
            adapter_live_background_poll_hz=_live_poll_hz(config),
        )
    if command_type == "start_session":
        if validate_start_session_payload(payload):
            return config
        profile = "stub"
        if isinstance(payload, dict):
            profile = str(payload.get("runtime_profile", "stub"))
        if profile == "mock_adapter":
            return replace(config, enable_gazebo_adapter=True, adapter_mode="mock")
        if profile == "live":
            return replace(
                config,
                enable_gazebo_adapter=True,
                adapter_mode="live",
                adapter_live_background_poll_hz=_live_poll_hz(config),
            )
        return config
    return config


def _live_poll_hz(config: GovernanceConfig) -> float:
    if config.adapter_live_background_poll_hz > 0:
        return config.adapter_live_background_poll_hz
    return LIVE_ADAPTER_BACKGROUND_POLL_HZ


def _config_for_session(config: GovernanceConfig, session: Any) -> GovernanceConfig:
    runtime = getattr(session, "runtime", None)
    if getattr(runtime, "kind", "") != "adapter":
        return config
    mode = getattr(runtime, "mode", config.adapter_mode)
    out = replace(
        config,
        enable_gazebo_adapter=True,
        adapter_mode=mode,
    )
    if mode == "live":
        out = replace(out, adapter_live_background_poll_hz=_live_poll_hz(config))
    return out


# Re-export for backward compatibility
__all__ = ["BridgeSessionManager", "GovernanceConfig", "SessionRecord"]


class BridgeSessionManager:
    def __init__(
        self,
        config: GovernanceConfig | None = None,
        repo_root: Path | None = None,
    ) -> None:
        self.config = config or GovernanceConfig()
        self._repo_root = repo_root
        self._registry = SessionRegistry()
        self._editing_session_id: str | None = None
        self._rate_limiters: dict[str, RateLimiter] = {}
        self._global_rate_limiter = RateLimiter(
            burst=self.config.command_rate_burst,
            sustained_per_s=self.config.command_rate_sustained,
        )
        self._audit = AuditLog(repo_root)
        self._export_audit = ExportAuditLog(repo_root)
        hz = self.config.telemetry_update_rate_cap_hz
        self._telemetry = TelemetryBuffer(max_hz=hz)
        self._telemetry_subs = TelemetrySubscriptionStore(
            max_hz=hz,
            ring_size=self.config.telemetry_ring_buffer_size,
        )

    @property
    def _session(self) -> SessionRecord | None:
        """Backward-compatible single-session accessor."""
        non_terminal = self._registry.iter_non_terminal()
        if non_terminal:
            return non_terminal[0]
        all_sessions = list(self._registry.iter_all())
        if len(all_sessions) == 1:
            return all_sessions[0]
        return None

    def handle_command(self, body: dict[str, Any]) -> dict[str, Any]:
        now = time.monotonic()
        raw_command_type = str(body.get("command_type", ""))
        command_type = _SIM_COMMAND_CANONICAL.get(raw_command_type, raw_command_type)
        command_id = str(body.get("command_id", ""))
        issued_by = str(body.get("issued_by", "rt_ui_prototype"))
        authority_scope = str(body.get("authority_scope", ""))
        session_id = body.get("session_id")
        payload = _payload_for_sim_alias(raw_command_type, body.get("payload"))

        base = response_base(command_id, session_id)

        if isinstance(session_id, str):
            pre_session = self._registry.get(session_id)
            if (
                pre_session is not None
                and pre_session.state in {SessionState.RUNNING, SessionState.PAUSED}
                and not pre_session.runtime.is_alive()
            ):
                return self._runtime_crashed(
                    pre_session,
                    base,
                    command_id,
                    issued_by,
                )

        self._tick_timeouts(now)
        self._evict_terminal_sessions()

        if authority_scope and authority_scope != self.config.authority_scope:
            return fail(base, "COMMAND_FORBIDDEN", "invalid authority_scope")

        forbidden = classify_command(raw_command_type)
        if forbidden:
            return fail(base, forbidden, f"command not allowed: {command_type}")

        cap_err = validate_capture_payload(command_type, payload)
        if cap_err:
            return fail(base, cap_err, "invalid capture payload")

        if command_type == "start_session":
            start_err = validate_start_session_payload(payload)
            if start_err:
                return fail(base, start_err, "invalid start_session payload")

        command_config = _config_for_command(self.config, raw_command_type, payload)

        if command_type == "start_session":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            profile = "stub"
            if isinstance(payload, dict):
                profile = str(payload.get("runtime_profile", "stub"))
            if profile == "live":
                preflight = check_live_runtime_preflight()
                if not preflight.get("ok"):
                    return fail(
                        base,
                        "RUNTIME_UNAVAILABLE",
                        str(preflight.get("message", "live runtime unavailable")),
                        preflight=preflight,
                    )
            result = start_session(
                base,
                config=command_config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_transition=self._publish_channels_for_transition,
                non_terminal_count=self._registry.non_terminal_count,
                register_session=self._register_session,
                command_id=command_id,
                issued_by=issued_by,
                now=now,
                runtime_profile=profile,
            )
            if result.get("ok") and result.get("session_id"):
                sid = str(result["session_id"])
                if self._editing_session_id is None:
                    self._editing_session_id = sid
            return result

        if command_type == "list_sessions":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            return handle_list_sessions(
                base,
                registry=self._registry,
                editing_session_id=self._editing_session_id,
                config=self.config,
            )

        if command_type == "set_editing_session":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            result = handle_set_editing_session(
                base,
                payload,
                registry=self._registry,
                audit=self._audit,
                command_id=command_id,
                issued_by=issued_by,
            )
            if result.get("ok") and result.get("editing_session_id"):
                self._editing_session_id = str(result["editing_session_id"])
            return result

        if command_type == "list_capture_handoff_status":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            return handle_list_capture_handoff_status(
                base,
                payload,
                repo_root=self._repo_root,
            )

        if command_type == "list_runtime_templates":
            return list_runtime_templates(base, self.config)

        if command_type == "check_live_runtime_preflight":
            if not self._global_rate_limiter.check(now):
                return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")
            preflight = check_live_runtime_preflight()
            resp = ok(base, state="idle")
            resp["preflight"] = preflight
            return resp

        sid = session_id
        if not sid or not isinstance(sid, str):
            return fail(base, "SESSION_NOT_FOUND", "unknown session_id")

        session = self._registry.get(sid)
        if session is None:
            return fail(base, "SESSION_NOT_FOUND", "unknown session_id")

        base["session_id"] = session.session_id
        session_config = _config_for_session(self.config, session)

        if not self._rate_limiter_for(session.session_id).check(now):
            return fail(base, "RESOURCE_LIMIT_EXCEEDED", "command rate limit")

        if command_type in ENTITY_COMMANDS or command_type in {
            "apply_runtime_template",
            "apply_scenario",
            "assign_target",
            "cancel_assignment",
            "start_capture",
            "stop_capture",
            "capture_status",
        }:
            if session.session_id != self._editing_session_id:
                return fail(
                    base,
                    "EDITING_SESSION_MISMATCH",
                    "entity mutations require editing session",
                )
        if command_type in TACTICAL_COMMANDS and command_type != "get_tactical_state":
            if session.session_id != self._editing_session_id:
                return fail(
                    base,
                    "EDITING_SESSION_MISMATCH",
                    "tactical mutations require editing session",
                )

        if command_type == "pause_session":
            return pause_session(
                session,
                base,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                runtime_crashed_fn=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "resume":
            return resume_session(
                session,
                base,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                runtime_crashed_fn=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "stop_session":
            return stop_session(
                session,
                base,
                config=session_config,
                audit=self._audit,
                publish_transition=self._publish_channels_for_transition,
                command_id=command_id,
                issued_by=issued_by,
                now=now,
                terminate_runtime=raw_command_type == "stop_sim",
            )
        if command_type == "discard_session":
            result = discard_session(
                session,
                base,
                audit=self._audit,
                export_audit=self._export_audit,
                telemetry_subs=self._telemetry_subs,
                command_id=command_id,
                issued_by=issued_by,
            )
            if result.get("ok"):
                self._unregister_session(session.session_id)
            return result
        if command_type == "capture_session":
            result = handle_capture_session(
                session,
                base,
                config=self.config,
                audit=self._audit,
                export_audit=self._export_audit,
                telemetry_subs=self._telemetry_subs,
                repo_root=self._repo_root,
                command_id=command_id,
                issued_by=issued_by,
                payload=payload,
            )
            if result.get("ok"):
                self._unregister_session(session.session_id)
            return result
        if command_type == "reset_session":
            return reset_session(
                session,
                base,
                config=session_config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_transition=self._publish_channels_for_transition,
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in {"start_capture", "stop_capture", "capture_status"}:
            return self._handle_runtime_capture(
                session,
                command_type,
                base,
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in {"assign_target", "cancel_assignment"}:
            return self._handle_live_assignment(
                session,
                command_type,
                payload,
                base,
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "apply_scenario":
            return self._handle_apply_scenario(
                session,
                payload,
                base,
                config=session_config,
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in ENTITY_COMMANDS:
            return handle_entity(
                session,
                command_type,
                payload,
                base,
                config=session_config,
                audit=self._audit,
                telemetry=self._telemetry,
                telemetry_subs=self._telemetry_subs,
                publish_channel=lambda ch: self._publish_telemetry(session, ch),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
                total_entity_count=self._registry.total_entity_count,
            )
        if command_type in TELEMETRY_COMMANDS:
            return handle_telemetry(
                session,
                command_type,
                payload,
                base,
                config=session_config,
                telemetry_subs=self._telemetry_subs,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                poll_bridge=lambda: poll_telemetry_bridge(
                    session,
                    session_config,
                    self._audit,
                    lambda ch: self._publish_telemetry(
                        session,
                        ch,
                        command_id=command_id,
                        issued_by=issued_by,
                    ),
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in TEMPLATE_COMMANDS:
            return handle_template(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in WORKFLOW_COMMANDS:
            return handle_workflow(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                    session, ct, st, skip_adapter_poll=skip
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type == "send_runtime_command":
            return handle_runtime_command(
                session,
                payload,
                base,
                config=session_config,
                audit=self._audit,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                runtime_crashed=lambda: self._runtime_crashed(
                    session, base, command_id, issued_by
                ),
                command_id=command_id,
                issued_by=issued_by,
            )
        if command_type in TACTICAL_COMMANDS:
            return handle_tactical(
                session,
                command_type,
                payload,
                base,
                config=self.config,
                audit=self._audit,
                telemetry=self._telemetry,
                publish_channel=lambda ch: self._publish_telemetry(
                    session,
                    ch,
                    command_id=command_id,
                    issued_by=issued_by,
                ),
                command_id=command_id,
                issued_by=issued_by,
            )

        return fail(base, "COMMAND_FORBIDDEN", command_type)

    def _handle_runtime_capture(
        self,
        session: SessionRecord,
        command_type: str,
        base: dict[str, Any],
        *,
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, command_type):
            return fail(base, "INVALID_STATE", session.state.value)
        if not session.runtime.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)

        if command_type == "capture_status":
            status = runtime_capture_status(session)
            resp = ok(base, state=session.state.value)
            resp.update(status)
            return resp

        if command_type == "start_capture":
            if session.runtime_capture is not None:
                return fail(base, "INVALID_STATE", "runtime capture already active")
            capture_state = begin_runtime_capture(session)
            self._audit.append(
                session.session_id,
                command_id=command_id,
                command_type="start_capture",
                issued_by=issued_by,
                result="OK",
                detail={
                    "capture_id": capture_state.capture_id,
                    "state": session.state.value,
                },
            )
            self._publish_channels_for_transition(session, "start_capture", session.state)
            self._publish_telemetry(
                session,
                "world_summary",
                command_id=command_id,
                issued_by=issued_by,
            )
            self._publish_telemetry(
                session,
                "entity_pose_mirror",
                command_id=command_id,
                issued_by=issued_by,
            )
            resp = ok(base, state=session.state.value)
            resp["capture_id"] = capture_state.capture_id
            resp["capture_active"] = True
            resp["started_utc"] = capture_state.started_utc
            resp["frames_count"] = len(capture_state.telemetry_frames)
            resp["entities_count"] = session.world.registry.count() if session.world else 0
            return resp

        if session.runtime_capture is None:
            return fail(base, "INVALID_STATE", "runtime capture not active")
        self._publish_channels_for_transition(session, "stop_capture", session.state)
        root = self._repo_root or repo_root_from()
        artifact, artifact_path = finalize_runtime_capture(
            session,
            repo_root=root,
            audit_path=self._audit.path_for(session.session_id),
        )
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="stop_capture",
            issued_by=issued_by,
            result="OK",
            detail={
                "capture_id": artifact["capture_id"],
                "artifact_ref": artifact_path.as_posix(),
                "telemetry_frame_count": len(artifact.get("telemetry_frames") or []),
                "state": session.state.value,
            },
        )
        resp = ok(base, state=session.state.value)
        resp["capture_id"] = artifact["capture_id"]
        resp["capture_active"] = False
        resp["artifact_ref"] = artifact_path.as_posix()
        resp["artifact_path"] = artifact_path.as_posix()
        validation = validate_runtime_capture_artifact(artifact)
        resp["artifact_schema"] = artifact.get("schema")
        resp["artifact_valid"] = validation["valid"]
        resp["telemetry_frame_count"] = len(artifact.get("telemetry_frames") or [])
        resp["frames_count"] = len(artifact.get("telemetry_frames") or [])
        resp["entities_count"] = len(artifact.get("entities") or [])
        resp["lifecycle_transition_count"] = len(artifact.get("lifecycle_transitions") or [])
        return resp

    def _handle_live_assignment(
        self,
        session: SessionRecord,
        command_type: str,
        payload: Any,
        base: dict[str, Any],
        *,
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not can_transition(session.state, command_type):
            return fail(base, "INVALID_STATE", session.state.value)
        if not session.runtime.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        if session.world is None:
            return fail(base, "INVALID_STATE", "world not initialized")
        if not isinstance(payload, dict):
            return fail(base, "INVALID_PAYLOAD", command_type)

        defender_id_raw = payload.get("defender_id")
        if not isinstance(defender_id_raw, str) or not defender_id_raw:
            return fail(base, "INVALID_PAYLOAD", "defender_id required")
        defender_id = defender_id_raw
        defender = session.world.registry.get(defender_id)
        if defender is None:
            return fail(base, "ENTITY_NOT_FOUND", "defender_id")
        if defender.entity_type != "interceptor":
            return fail(base, "COMMAND_FORBIDDEN", "defender must be interceptor")

        previous_target_id = session.live_assignments.get(defender_id)
        if command_type == "assign_target":
            target_id_raw = payload.get("target_id")
            if not isinstance(target_id_raw, str) or not target_id_raw:
                return fail(base, "INVALID_PAYLOAD", "target_id required")
            target_id = target_id_raw
            target = session.world.registry.get(target_id)
            if target is None:
                return fail(base, "ENTITY_NOT_FOUND", "target_id")
            if target.entity_type != "drone":
                return fail(base, "COMMAND_FORBIDDEN", "target must be drone")
            session.live_assignments[defender_id] = target_id
            detail = {
                "defender_id": defender_id,
                "target_id": target_id,
                "previous_target_id": previous_target_id,
                "assignment_state": "assigned",
                "state": session.state.value,
            }
            result_target_id = target_id
            assignment_state = "assigned"
        else:
            removed_target_id = session.live_assignments.pop(defender_id, None)
            detail = {
                "defender_id": defender_id,
                "target_id": removed_target_id,
                "previous_target_id": previous_target_id,
                "assignment_state": "cleared",
                "state": session.state.value,
            }
            result_target_id = removed_target_id
            assignment_state = "cleared"

        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type=command_type,
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        self._publish_channels_for_transition(session, command_type, session.state)
        resp = ok(base, state=session.state.value)
        resp["defender_id"] = defender_id
        resp["target_id"] = result_target_id
        resp["active_target_id"] = session.live_assignments.get(defender_id)
        resp["assignment_state"] = assignment_state
        resp["assignment_count"] = len(session.live_assignments)
        return resp

    def _handle_apply_scenario(
        self,
        session: SessionRecord,
        payload: Any,
        base: dict[str, Any],
        *,
        config: GovernanceConfig,
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        if not session.runtime.is_alive():
            return self._runtime_crashed(session, base, command_id, issued_by)
        if session.world is None:
            return fail(base, "INVALID_STATE", "world not initialized")
        if not session.state:
            return fail(base, "INVALID_STATE", "session state unavailable")

        normalized, payload_err = _validate_scenario_payload(payload, config=config)
        if payload_err:
            return fail(base, payload_err, "invalid payload for apply_scenario")
        assert normalized is not None

        existing_count = session.world.registry.count()
        scenario_total = int(normalized["total_entity_count"])
        aggregate_after_reset = (
            self._registry.total_entity_count() - existing_count + scenario_total
        )
        if aggregate_after_reset > config.max_total_entities_across_sessions:
            return fail(
                base,
                "RESOURCE_LIMIT_EXCEEDED",
                "aggregate entity cap across sessions",
            )

        reset_resp = reset_session(
            session,
            base,
            config=config,
            audit=self._audit,
            telemetry=self._telemetry,
            publish_transition=self._publish_channels_for_transition,
            command_id=command_id,
            issued_by=issued_by,
        )
        if not reset_resp.get("ok"):
            return reset_resp

        counts = {"assets": 0, "defenders": 0, "attackers": 0}
        entity_ids: dict[str, list[str]] = {"assets": [], "defenders": [], "attackers": []}

        def spawn_payload(group: str, item: dict[str, Any]) -> dict[str, Any]:
            if group == "defenders":
                return _payload_for_sim_alias("spawn_defender", item)
            if group == "attackers":
                return _payload_for_sim_alias("spawn_attacker", item)
            out = {"entity_type": item["entity_type"], "pose": item["pose"]}
            if item.get("entity_id"):
                out["entity_id"] = item["entity_id"]
            return out

        for group in ("assets", "defenders", "attackers"):
            for item in normalized[group]:
                spawn_resp = handle_entity(
                    session,
                    "spawn_entity",
                    spawn_payload(group, item),
                    base,
                    config=config,
                    audit=self._audit,
                    telemetry=self._telemetry,
                    telemetry_subs=self._telemetry_subs,
                    publish_channel=lambda ch: self._publish_telemetry(session, ch),
                    publish_transition=lambda ct, st, skip: self._publish_channels_for_transition(
                        session, ct, st, skip_adapter_poll=skip
                    ),
                    runtime_crashed=lambda: self._runtime_crashed(
                        session, base, command_id, issued_by
                    ),
                    command_id=command_id,
                    issued_by=issued_by,
                    total_entity_count=self._registry.total_entity_count,
                )
                if not spawn_resp.get("ok"):
                    self._audit.append(
                        session.session_id,
                        command_id=command_id,
                        command_type="apply_scenario",
                        issued_by=issued_by,
                        result=str(spawn_resp.get("error_code") or "INVALID_STATE"),
                        detail={"failed_group": group, "state": session.state.value},
                    )
                    return spawn_resp
                counts[group] += 1
                if spawn_resp.get("entity_id"):
                    entity_ids[group].append(str(spawn_resp["entity_id"]))

        detail = {
            "terrain_preset": normalized["terrain_preset"],
            "counts": dict(counts),
            "entity_ids": entity_ids,
            "state": session.state.value,
        }
        self._audit.append(
            session.session_id,
            command_id=command_id,
            command_type="apply_scenario",
            issued_by=issued_by,
            result="OK",
            detail=detail,
        )
        self._publish_channels_for_transition(session, "apply_scenario", session.state)
        resp = ok(base, state=session.state.value)
        resp["terrain_preset"] = normalized["terrain_preset"]
        resp["counts"] = counts
        resp["asset_count"] = counts["assets"]
        resp["defender_count"] = counts["defenders"]
        resp["attacker_count"] = counts["attackers"]
        resp["entity_ids"] = entity_ids
        resp["world_summary"] = session.world.world_summary()
        return resp

    def _rate_limiter_for(self, session_id: str) -> RateLimiter:
        if session_id not in self._rate_limiters:
            self._rate_limiters[session_id] = RateLimiter(
                burst=self.config.command_rate_burst,
                sustained_per_s=self.config.command_rate_sustained,
            )
        return self._rate_limiters[session_id]

    def _register_session(self, record: SessionRecord) -> None:
        self._registry.register(record)
        _ = self._rate_limiter_for(record.session_id)

    def _unregister_session(self, session_id: str) -> None:
        self._registry.evict(session_id)
        self._rate_limiters.pop(session_id, None)
        self._editing_session_id = pick_editing_session_after_evict(
            self._registry,
            session_id,
            self._editing_session_id,
        )

    def _evict_terminal_sessions(self) -> None:
        for session in list(self._registry.iter_all()):
            if session.state in TERMINAL_STATES:
                self._unregister_session(session.session_id)

    def pull_telemetry(
        self,
        session_id: str,
        subscription_id: str,
        max_events: int = 10,
    ) -> dict[str, Any]:
        self._tick_timeouts(time.monotonic())
        sub = self._telemetry_subs.get(subscription_id)
        if sub is None or sub.session_id != session_id:
            return {
                "governance_banner": GOVERNANCE_BANNER,
                "ok": False,
                "error_code": "SESSION_NOT_FOUND",
                "message": "unknown subscription_id",
                "events": [],
                "drained_count": 0,
            }
        events = self._telemetry_subs.drain(subscription_id, max_events=max_events)
        return {
            "governance_banner": GOVERNANCE_BANNER,
            "ok": True,
            "error_code": "OK",
            "session_id": session_id,
            "subscription_id": subscription_id,
            "events": events,
            "drained_count": len(events),
        }

    def _publish_telemetry(
        self,
        session: SessionRecord,
        channel: str,
        *,
        command_type: str | None = None,
        previous_state: str | None = None,
        command_id: str | None = None,
        issued_by: str = "bridge",
    ) -> None:
        publish_telemetry(
            session,
            channel,
            config=_config_for_session(self.config, session),
            telemetry_subs=self._telemetry_subs,
            audit=self._audit,
            command_type=command_type,
            previous_state=previous_state,
            command_id=command_id,
            issued_by=issued_by,
        )

    def _publish_channels_for_transition(
        self,
        session: SessionRecord,
        command_type: str,
        previous_state: SessionState,
        *,
        skip_adapter_poll: bool = False,
    ) -> None:
        publish_channels_for_transition(
            session,
            command_type,
            previous_state,
            config=_config_for_session(self.config, session),
            telemetry_subs=self._telemetry_subs,
            audit=self._audit,
            publish_channel=lambda ch: self._publish_telemetry(session, ch),
            skip_adapter_poll=skip_adapter_poll,
        )

    def _runtime_crashed(
        self,
        session: SessionRecord,
        base: dict[str, Any],
        command_id: str,
        issued_by: str,
    ) -> dict[str, Any]:
        return lifecycle_runtime_crashed(
            session,
            base,
            self._audit,
            self.config,
            command_id=command_id,
            issued_by=issued_by,
        )

    def _tick_timeouts(self, now: float) -> None:
        mono = monotonic_now()
        for session in self._registry.iter_non_terminal():
            tick_timeouts(
                session,
                config=self.config,
                audit=self._audit,
                telemetry_subs=self._telemetry_subs,
                now=now,
            )
            self._tick_live_background_poll(session, now)
            tick_tactical_autonomous_for_session(
                session,
                mono,
                config=self.config,
                audit=self._audit,
                publish_channel=lambda ch, s=session: self._publish_telemetry(s, ch),
            )

    def _tick_live_background_poll(self, session: SessionRecord, now: float) -> None:
        session_config = _config_for_session(self.config, session)
        hz = session_config.adapter_live_background_poll_hz
        if hz <= 0 or getattr(session.runtime, "mode", None) != "live":
            return
        if session.state not in {SessionState.RUNNING, SessionState.PAUSED}:
            return
        if not runtime_is_adapter(session.runtime) or not session.runtime.is_alive():
            return
        last = session.last_live_background_poll_monotonic
        if last is not None and (now - last) < (1.0 / hz):
            return
        session.last_live_background_poll_monotonic = now
        from datetime import datetime, timezone

        session.last_live_background_poll_utc = (
            datetime.now(timezone.utc).replace(microsecond=0).isoformat()
        )
        poll_result = run_adapter_poll_tick(
            session,
            session_config,
            poll_feedback=True,
            poll_telemetry=True,
            emit_feedback_audit=False,
        )
        apply_adapter_poll_result(
            session,
            poll_result,
            self._audit,
            lambda ch: self._publish_telemetry(session, ch),
            issued_by="bridge_live_poll",
        )

    # Backward-compatible private aliases for tests
    def _response_base(self, command_id: str, session_id: Any) -> dict[str, Any]:
        return response_base(command_id, session_id)

    def _ok(self, base: dict[str, Any], *, state: str) -> dict[str, Any]:
        return ok(base, state=state)

    def _fail(self, base: dict[str, Any], error_code: str, message: str) -> dict[str, Any]:
        return fail(base, error_code, message)
