"""Adapter-fed telemetry mirror (PLAT-RT-G4).

Revision counters: docs/evaluation/rt_revision_vocabulary_v1.md
Authority labels: docs/evaluation/rt_authority_model_v1.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Literal

from rt_sandbox.authority_labels import (
    AUTHORITY_COMMAND,
    AUTHORITY_EXPLANATORY_TELEMETRY,
    SOURCE_ADAPTER_FEEDBACK,
    SOURCE_ADAPTER_TELEMETRY,
    SOURCE_BRIDGE_REGISTRY,
    SOURCE_BRIDGE_SESSION,
    enrich_channel_payload,
)
from rt_sandbox.governance import GovernanceConfig, LIVE_ADAPTER_BACKGROUND_POLL_HZ
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.time_utils import is_poll_stale

TelemetryHealth = Literal["ok", "stale", "feedback_lost"]


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def telemetry_bridge_active(session: Any, config: GovernanceConfig) -> bool:
    if not config.enable_gazebo_adapter or not config.telemetry_bridge_enabled:
        return False
    return runtime_is_adapter(session.runtime)


@dataclass
class TelemetryMirror:
    session_id: str
    telemetry_revision: int = 0
    last_poll_utc: str | None = None
    telemetry_health: TelemetryHealth = "ok"
    clock_mirror: dict[str, Any] = field(default_factory=dict)
    adapter_health: dict[str, Any] = field(default_factory=dict)
    entity_pose_mirror: dict[str, Any] = field(default_factory=lambda: {"entities": []})
    world_revision_hint: dict[str, Any] = field(default_factory=dict)
    fidelity_truth: dict[str, Any] | None = None

    def clear(self) -> None:
        self.telemetry_revision = 0
        self.last_poll_utc = None
        self.telemetry_health = "ok"
        self.clock_mirror = {}
        self.adapter_health = {}
        self.entity_pose_mirror = {"entities": []}
        self.world_revision_hint = {}
        self.fidelity_truth = None

    def update_from_poll(self, bundle: dict[str, Any], config: GovernanceConfig) -> str:
        ts = str(bundle.get("timestamp_utc") or _utc_now())
        self.last_poll_utc = ts
        self.telemetry_revision = int(bundle.get("telemetry_seq") or 0)
        self.clock_mirror = dict(bundle.get("clock_mirror") or {})
        self.adapter_health = dict(bundle.get("adapter_health") or {})
        self.entity_pose_mirror = dict(bundle.get("entity_pose_mirror") or {"entities": []})
        self.world_revision_hint = dict(bundle.get("world_revision_hint") or {})
        ft = bundle.get("fidelity_truth")
        self.fidelity_truth = dict(ft) if isinstance(ft, dict) else None
        self.telemetry_health = "ok"
        if self.check_stale(config):
            self.telemetry_health = "stale"
            return "telemetry_stale"
        return "telemetry_update"

    def check_stale(self, config: GovernanceConfig) -> bool:
        return is_poll_stale(self.last_poll_utc, config.telemetry_stale_s)

    def summary(self) -> dict[str, Any]:
        """Explanatory snapshot for capture normalization (PLAT-RT-G5)."""
        out: dict[str, Any] = {
            "telemetry_revision": self.telemetry_revision,
            "telemetry_health": self.telemetry_health,
            "last_poll_utc": self.last_poll_utc,
            "clock_mirror": dict(self.clock_mirror),
            "adapter_health": dict(self.adapter_health),
            "entity_pose_mirror": dict(self.entity_pose_mirror),
            "world_revision_hint": dict(self.world_revision_hint),
        }
        if self.fidelity_truth is not None:
            out["fidelity_truth"] = dict(self.fidelity_truth)
        return out


def new_telemetry_mirror(session_id: str) -> TelemetryMirror:
    return TelemetryMirror(session_id=session_id)


def ensure_telemetry_mirror(session: Any) -> TelemetryMirror:
    mirror = getattr(session, "telemetry_mirror", None)
    if mirror is None:
        mirror = new_telemetry_mirror(session.session_id)
        session.telemetry_mirror = mirror
    return mirror


def clear_telemetry_mirror(session: Any) -> None:
    mirror = getattr(session, "telemetry_mirror", None)
    if mirror is not None:
        mirror.clear()


def _entity_telemetry_entry(
    entry: dict[str, Any],
    lifecycle_state: str,
    assignments: dict[str, str],
    assigned_targets: set[str],
) -> dict[str, Any]:
    out = dict(entry)
    pose = dict(out.get("pose") or out.get("position") or {})
    position = {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
    }
    velocity_in = dict(out.get("velocity") or {})
    vx = float(velocity_in.get("x", velocity_in.get("vx", 0.0)))
    vy = float(velocity_in.get("y", velocity_in.get("vy", 0.0)))
    vz = float(velocity_in.get("z", velocity_in.get("vz", 0.0)))
    speed = float(velocity_in.get("speed_mps", (vx * vx + vy * vy + vz * vz) ** 0.5))
    heading = out.get("heading_deg")
    if heading is None:
        heading = pose.get("yaw_deg", 0.0)
    out["position"] = position
    out["velocity"] = {"x": vx, "y": vy, "z": vz, "speed_mps": speed}
    out["speed_mps"] = speed
    out["heading_deg"] = float(heading)
    entity_id = str(out.get("entity_id") or "")
    active_target_id = assignments.get(entity_id)
    if active_target_id:
        out["active_target_id"] = active_target_id
        out["assignment_state"] = "assigned"
    else:
        out["assignment_state"] = str(out.get("assignment_state") or "none")
    if entity_id in assigned_targets:
        out["target_state"] = "assigned"
    else:
        out["target_state"] = str(out.get("target_state") or "none")
    out["lifecycle_state"] = lifecycle_state
    return out


def _with_entity_telemetry_fields(
    payload: dict[str, Any], session: Any
) -> dict[str, Any]:
    out = dict(payload)
    lifecycle_state = str(getattr(getattr(session, "state", None), "value", "unknown"))
    assignments = dict(getattr(session, "live_assignments", {}) or {})
    assigned_targets = set(assignments.values())
    out["entities"] = [
        _entity_telemetry_entry(item, lifecycle_state, assignments, assigned_targets)
        if isinstance(item, dict)
        else item
        for item in list(out.get("entities") or [])
    ]
    return out


def poll_adapter_telemetry(
    runtime: Any,
    config: GovernanceConfig,
    *,
    mock_stale_telemetry: bool = False,
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not config.telemetry_bridge_enabled:
        return None
    if not runtime_is_adapter(runtime):
        return None
    fn = getattr(runtime, "poll_telemetry", None)
    if fn is None:
        return None
    kwargs: dict[str, Any] = {"mock_stale_telemetry": mock_stale_telemetry}
    if config.enable_fidelity_coupling:
        kwargs["enable_fidelity_coupling"] = True
    return fn(**kwargs)


def poll_and_update_mirror(
    session: Any,
    config: GovernanceConfig,
    *,
    mock_stale_telemetry: bool = False,
) -> tuple[str | None, str | None, dict[str, Any] | None]:
    """Poll adapter, update TelemetryMirror. Returns audit_event, error_code, detail."""
    if not telemetry_bridge_active(session, config):
        return None, None, None
    bundle = poll_adapter_telemetry(
        session.runtime,
        config,
        mock_stale_telemetry=mock_stale_telemetry,
    )
    if bundle is None:
        return None, None, None
    if bundle.get("error_code"):
        mirror = ensure_telemetry_mirror(session)
        mirror.telemetry_health = "feedback_lost"
        return (
            "telemetry_feedback_lost",
            str(bundle["error_code"]),
            dict(bundle),
        )
    mirror = ensure_telemetry_mirror(session)
    audit_event = mirror.update_from_poll(bundle, config)
    detail: dict[str, Any] = {
        "telemetry_revision": mirror.telemetry_revision,
        "telemetry_health": mirror.telemetry_health,
        "last_poll_utc": mirror.last_poll_utc,
    }
    return audit_event, None, detail


def _merge_fidelity_telemetry_fields(
    payload: dict[str, Any],
    session: Any,
    config: GovernanceConfig,
) -> dict[str, Any]:
    from rt_sandbox.fidelity_coupling import build_fidelity_telemetry_fields

    merged = dict(payload)
    merged.update(build_fidelity_telemetry_fields(session, config))
    return merged


def _session_health_live_fields(
    session: Any,
    config: GovernanceConfig | None,
) -> dict[str, Any]:
    profile = str(getattr(session, "runtime_profile", "stub"))
    out: dict[str, Any] = {"runtime_profile": profile}
    runtime = getattr(session, "runtime", None)
    mode = getattr(runtime, "mode", None)
    if profile != "live" and mode != "live":
        return out
    hz = 0.0
    if config is not None and config.adapter_live_background_poll_hz > 0:
        hz = float(config.adapter_live_background_poll_hz)
    elif mode == "live":
        hz = float(LIVE_ADAPTER_BACKGROUND_POLL_HZ)
    out["live_background_poll_hz"] = hz
    last_utc = getattr(session, "last_live_background_poll_utc", None)
    if isinstance(last_utc, str) and last_utc:
        out["last_live_poll_utc"] = last_utc
    return out


def _build_stub_channel_payload(
    session: Any,
    channel: str,
    config: GovernanceConfig | None = None,
) -> dict[str, Any] | None:
    from rt_sandbox.lifecycle import SessionState

    if channel == "session_health":
        health = session.runtime.health_payload()
        payload = enrich_channel_payload(
            {
                "state": session.state.value,
                "stub_alive": health.get("stub_alive", False),
                "adapter_alive": health.get("adapter_alive", False),
                "adapter_mode": health.get("adapter_mode"),
                "adapter_pid": health.get("adapter_pid"),
                **_session_health_live_fields(session, config),
            },
            source=SOURCE_BRIDGE_SESSION,
            authority_label=AUTHORITY_COMMAND,
        )
        if config is not None:
            return _merge_fidelity_telemetry_fields(payload, session, config)
        return payload
    if channel == "lifecycle_state":
        return enrich_channel_payload(
            {
                "state": session.state.value,
                "previous_state": None,
                "command_type": None,
            },
            source=SOURCE_BRIDGE_SESSION,
            authority_label=AUTHORITY_COMMAND,
        )
    if channel == "clock_mirror":
        return enrich_channel_payload(
            {"paused": session.state == SessionState.PAUSED},
            source=SOURCE_BRIDGE_SESSION,
            authority_label=AUTHORITY_COMMAND,
        )
    if channel == "world_summary":
        if session.world is None:
            return enrich_channel_payload(
                {"entity_count": 0, "revision": 0, "by_type": {}, "bounds": {}},
                source=SOURCE_BRIDGE_REGISTRY,
                authority_label=AUTHORITY_COMMAND,
            )
        pose_sync_summary = None
        if session.pose_sync is not None and session.world is not None:
            pose_sync_summary = session.pose_sync.summary(session.world.revision)
        summary = session.world.world_summary(pose_sync_summary=pose_sync_summary)
        payload = enrich_channel_payload(
            summary,
            source=SOURCE_BRIDGE_REGISTRY,
            authority_label=AUTHORITY_COMMAND,
        )
        if config is not None:
            return _merge_fidelity_telemetry_fields(payload, session, config)
        return payload
    if channel == "entity_pose_mirror":
        if session.world is None:
            return enrich_channel_payload(
                {"entities": []},
                source=SOURCE_BRIDGE_REGISTRY,
                authority_label=AUTHORITY_COMMAND,
            )
        payload = _with_entity_telemetry_fields(
            {"entities": session.world.registry.poses_for_telemetry()},
            session,
        )
        return enrich_channel_payload(
            payload,
            source=SOURCE_BRIDGE_REGISTRY,
            authority_label=AUTHORITY_COMMAND,
        )
    if channel == "tactical_state":
        from rt_sandbox.tactical_telemetry import build_tactical_state_payload

        return build_tactical_state_payload(session)
    if channel == "tactical_recommendation":
        from rt_sandbox.tactical_telemetry import build_tactical_recommendation_payload

        return build_tactical_recommendation_payload(session)
    if channel == "intelligence_advisory":
        from rt_sandbox.rt_intelligence_advisory_transport import (
            build_intelligence_advisory_transport,
        )

        stale_s = config.telemetry_stale_s if config is not None else 30.0
        return build_intelligence_advisory_transport(
            session, telemetry_stale_s=stale_s
        )
    return None


def resolve_channel_payload(
    session: Any,
    channel: str,
    config: GovernanceConfig,
    *,
    pose_sync_summary: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    if not telemetry_bridge_active(session, config):
        return _build_stub_channel_payload(session, channel, config)

    mirror = getattr(session, "telemetry_mirror", None)
    if mirror is None or mirror.last_poll_utc is None:
        return _build_stub_channel_payload(session, channel, config)

    from rt_sandbox.lifecycle import SessionState

    if channel == "lifecycle_state":
        return _build_stub_channel_payload(session, channel, config)
    if channel == "clock_mirror":
        payload = dict(mirror.clock_mirror)
        payload["telemetry_health"] = mirror.telemetry_health
        payload["telemetry_revision"] = mirror.telemetry_revision
        return enrich_channel_payload(
            payload,
            source=SOURCE_ADAPTER_TELEMETRY,
            authority_label=AUTHORITY_EXPLANATORY_TELEMETRY,
        )
    if channel == "entity_pose_mirror":
        payload = _with_entity_telemetry_fields(dict(mirror.entity_pose_mirror), session)
        payload["telemetry_health"] = mirror.telemetry_health
        payload["telemetry_revision"] = mirror.telemetry_revision
        return enrich_channel_payload(
            payload,
            source=SOURCE_ADAPTER_FEEDBACK,
            authority_label=AUTHORITY_EXPLANATORY_TELEMETRY,
        )
    if channel == "session_health":
        base = _build_stub_channel_payload(session, channel, config) or {}
        ah = mirror.adapter_health
        base["adapter_alive"] = ah.get("alive", base.get("adapter_alive"))
        base["adapter_entity_count"] = ah.get("entity_count")
        base["telemetry_health"] = mirror.telemetry_health
        base["telemetry_revision"] = mirror.telemetry_revision
        base["source"] = SOURCE_ADAPTER_TELEMETRY
        base["authority_label"] = AUTHORITY_EXPLANATORY_TELEMETRY
        base.update(_session_health_live_fields(session, config))
        return _merge_fidelity_telemetry_fields(base, session, config)
    if channel == "world_summary":
        if session.world is None:
            return enrich_channel_payload(
                {"entity_count": 0, "revision": 0, "by_type": {}, "bounds": {}},
                source=SOURCE_BRIDGE_REGISTRY,
                authority_label=AUTHORITY_COMMAND,
            )
        ps = pose_sync_summary
        if ps is None and session.pose_sync is not None:
            ps = session.pose_sync.summary(session.world.revision)
        summary = session.world.world_summary(pose_sync_summary=ps)
        summary["telemetry_revision"] = mirror.telemetry_revision
        summary["telemetry_health"] = mirror.telemetry_health
        payload = enrich_channel_payload(
            summary,
            source=SOURCE_BRIDGE_REGISTRY,
            authority_label=AUTHORITY_COMMAND,
        )
        return _merge_fidelity_telemetry_fields(payload, session, config)
    if channel == "tactical_state":
        from rt_sandbox.tactical_telemetry import build_tactical_state_payload

        return build_tactical_state_payload(session)
    if channel == "tactical_recommendation":
        from rt_sandbox.tactical_telemetry import build_tactical_recommendation_payload

        return build_tactical_recommendation_payload(session)
    if channel == "intelligence_advisory":
        from rt_sandbox.rt_intelligence_advisory_transport import (
            build_intelligence_advisory_transport,
        )

        return build_intelligence_advisory_transport(
            session, telemetry_stale_s=config.telemetry_stale_s
        )
    return _build_stub_channel_payload(session, channel, config)


def publish_all_telemetry_channels(
    session: Any,
    publish_fn: Callable[[str], None],
) -> None:
    for ch in (
        "session_health",
        "clock_mirror",
        "world_summary",
        "entity_pose_mirror",
        "intelligence_advisory",
    ):
        publish_fn(ch)
