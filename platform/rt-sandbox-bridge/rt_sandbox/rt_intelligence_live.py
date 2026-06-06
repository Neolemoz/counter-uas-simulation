"""Live-backed input assembly for recommendation-only intelligence advisory."""

from __future__ import annotations

import math
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.rt_intelligence_advisory_engine import INPUT_SCHEMA
from rt_sandbox.tactical_geometry import compute_intercept
from rt_sandbox.time_utils import is_poll_stale

ATTACKER_TYPE = "drone"
DEFENDER_TYPE = "interceptor"


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _position(pose: Any) -> dict[str, float] | None:
    if not isinstance(pose, dict):
        return None
    values = [_finite_number(pose.get(axis)) for axis in ("x", "y", "z")]
    if any(value is None for value in values):
        return None
    return dict(zip(("x", "y", "z"), values, strict=True))


def _stale(reason: str) -> tuple[None, str]:
    return None, reason


def _raw_velocity_rows(mirror: Any) -> dict[str, list[dict[str, Any]]]:
    raw = getattr(mirror, "entity_pose_mirror", {}) or {}
    rows: dict[str, list[dict[str, Any]]] = {}
    for item in list(raw.get("entities") or []):
        if not isinstance(item, dict):
            continue
        entity_id = item.get("entity_id")
        if isinstance(entity_id, str) and entity_id:
            rows.setdefault(entity_id, []).append(deepcopy(item))
    return rows


def _velocity(
    row: dict[str, Any], telemetry_revision: int
) -> tuple[dict[str, float] | None, str | None]:
    row_revision = row.get("telemetry_revision")
    if row_revision is not None and row_revision != telemetry_revision:
        return None, "attacker_velocity_stale"
    raw = row.get("velocity")
    if not isinstance(raw, dict) or any(axis not in raw for axis in ("x", "y", "z")):
        return None, "attacker_velocity_missing"
    values = [_finite_number(raw.get(axis)) for axis in ("x", "y", "z")]
    if any(value is None for value in values):
        return None, "attacker_velocity_nonfinite"
    return dict(zip(("x", "y", "z"), values, strict=True)), None


def _speed_cap(session: Any) -> float | None:
    tactical = getattr(session, "tactical", None)
    state = getattr(tactical, "state", None)
    return _finite_number(getattr(state, "interceptor_speed_cap_m_s", None))


def assemble_live_advisory_input(
    session: Any,
    *,
    telemetry_stale_s: float,
    now_utc: str | None = None,
) -> tuple[dict[str, Any] | None, str | None]:
    """Capture and validate one all-or-nothing live advisory snapshot."""
    world = getattr(session, "world", None)
    registry = getattr(world, "registry", None)
    center_id = getattr(session, "protected_center_entity_id", None)
    center = registry.get(center_id) if registry is not None and center_id else None
    center_position = _position(getattr(center, "pose", None))
    if center_position is None:
        return _stale("protected_center_unavailable")

    mirror = getattr(session, "telemetry_mirror", None)
    health = getattr(mirror, "telemetry_health", None)
    if health == "feedback_lost":
        return _stale("telemetry_feedback_lost")
    if (
        mirror is None
        or health != "ok"
        or is_poll_stale(getattr(mirror, "last_poll_utc", None), telemetry_stale_s)
    ):
        return _stale("telemetry_stale")

    world_revision = getattr(world, "revision", None)
    telemetry_revision = getattr(mirror, "telemetry_revision", None)
    lifecycle_state = str(getattr(getattr(session, "state", None), "value", ""))
    snapshot_invalid = (
        lifecycle_state not in {"running", "paused"}
        or not isinstance(world_revision, int)
        or not isinstance(telemetry_revision, int)
    )
    entities = [
        {
            "entity_id": record.entity_id,
            "entity_type": record.entity_type,
            "pose": dict(record.pose),
        }
        for record in registry.all_entities()
    ]
    entity_ids = [record["entity_id"] for record in entities]
    snapshot_invalid = snapshot_invalid or len(entity_ids) != len(set(entity_ids))
    attackers = sorted(
        (record for record in entities if record["entity_type"] == ATTACKER_TYPE),
        key=lambda record: record["entity_id"],
    )
    defenders = sorted(
        (record for record in entities if record["entity_type"] == DEFENDER_TYPE),
        key=lambda record: record["entity_id"],
    )
    velocity_rows = _raw_velocity_rows(mirror)

    attacker_positions: dict[str, dict[str, float]] = {}
    velocities: dict[str, dict[str, float]] = {}
    velocity_reasons: set[str] = set()
    for attacker in attackers:
        position = _position(attacker["pose"])
        if position is None:
            snapshot_invalid = True
        else:
            attacker_positions[attacker["entity_id"]] = position
        matches = velocity_rows.get(attacker["entity_id"], [])
        if len(matches) == 0:
            velocity_reasons.add("attacker_velocity_missing")
            continue
        if len(matches) != 1:
            snapshot_invalid = True
            continue
        velocity, reason = _velocity(matches[0], telemetry_revision)
        if reason:
            velocity_reasons.add(reason)
        elif velocity is not None:
            velocities[attacker["entity_id"]] = velocity

    for reason in (
        "attacker_velocity_missing",
        "attacker_velocity_stale",
        "attacker_velocity_nonfinite",
    ):
        if reason in velocity_reasons:
            return _stale(reason)

    cap = _speed_cap(session)
    defender_positions: dict[str, dict[str, float]] = {}
    defender_invalid = bool(defenders and (cap is None or cap <= 0.0))
    for defender in defenders:
        position = _position(defender["pose"])
        if position is None:
            defender_invalid = True
        else:
            defender_positions[defender["entity_id"]] = position
    if defender_invalid:
        return _stale("defender_invalid")
    if snapshot_invalid:
        return _stale("snapshot_validation_failed")

    attacker_rows: list[dict[str, Any]] = []
    for attacker in attackers:
        position = attacker_positions[attacker["entity_id"]]
        velocity = velocities[attacker["entity_id"]]
        attacker_rows.append(
            {
                "attacker_id": attacker["entity_id"],
                "position": position,
                "distance_to_protected_center_m": math.hypot(
                    position["x"] - center_position["x"],
                    position["y"] - center_position["y"],
                ),
                "descent_rate_mps": velocity["z"],
            }
        )
    defender_rows = [
        {
            "defender_id": defender["entity_id"],
            "position": defender_positions[defender["entity_id"]],
        }
        for defender in defenders
    ]
    feasible_tti: list[dict[str, Any]] = []
    for attacker in attackers:
        ap = attacker_positions[attacker["entity_id"]]
        velocity = velocities[attacker["entity_id"]]
        for defender in defenders:
            dp = defender_positions[defender["entity_id"]]
            result = compute_intercept(
                ap["x"], ap["y"], ap["z"],
                velocity["x"], velocity["y"], velocity["z"],
                dp["x"], dp["y"], dp["z"], float(cap),
            )
            feasible_tti.append(
                {
                    "attacker_id": attacker["entity_id"],
                    "defender_id": defender["entity_id"],
                    "feasible": result is not None,
                    "tti_s": result[0] if result is not None else None,
                    "reason": "feasible" if result is not None else "no_solution",
                }
            )

    if world.revision != world_revision or mirror.telemetry_revision != telemetry_revision:
        return _stale("snapshot_validation_failed")
    return (
        {
            "schema": INPUT_SCHEMA,
            "session_id": str(session.session_id),
            "advisory_utc": str(now_utc or _utc_now()),
            "attackers": attacker_rows,
            "defender_candidates": defender_rows,
            "feasible_tti": feasible_tti,
        },
        None,
    )
