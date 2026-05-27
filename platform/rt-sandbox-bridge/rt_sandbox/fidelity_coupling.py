"""Runtime fidelity coupling helpers (PLAT-RT-F5b P0).

See docs/evaluation/rt_runtime_fidelity_coupling_v1.md
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.pose_sync import pose_drift_m
from rt_sandbox.runtime_handle import runtime_is_adapter
from rt_sandbox.time_utils import is_poll_stale

FIDELITY_BANNER = (
    "RT FIDELITY TRUTH — sim-scoped attestation only; "
    "not SA replay or operational sensor authority"
)

FIDELITY_SCHEMA = "rt_fidelity_truth_snapshot_v1"
NOMINAL_DOME_RADIUS_M = 200.0
LOS_BLOCK_SEPARATION_M = 150.0


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def fidelity_coupling_active(session: Any, config: GovernanceConfig) -> bool:
    if not config.enable_fidelity_coupling:
        return False
    if not config.enable_gazebo_adapter:
        return False
    return runtime_is_adapter(session.runtime)


def _mock_los_label(entities: list[dict[str, Any]]) -> dict[str, Any]:
    if len(entities) <= 2:
        label = "clear"
    else:
        max_sep = 0.0
        for i, a in enumerate(entities):
            pa = a.get("truth_attested_pose") or a.get("pose") or {}
            for b in entities[i + 1 :]:
                pb = b.get("truth_attested_pose") or b.get("pose") or {}
                try:
                    dx = float(pa.get("x", 0)) - float(pb.get("x", 0))
                    dy = float(pa.get("y", 0)) - float(pb.get("y", 0))
                    max_sep = max(max_sep, math.hypot(dx, dy))
                except (TypeError, ValueError):
                    continue
        label = "clear" if max_sep < LOS_BLOCK_SEPARATION_M else "terrain_blocked"
    pair_ids = [str(e.get("entity_id", "")) for e in entities[:2] if e.get("entity_id")]
    return {"label": label, "pair_entity_ids": pair_ids}


def _mock_dome_truth(entities: list[dict[str, Any]]) -> dict[str, Any]:
    count = 0
    if entities:
        origin = entities[0].get("truth_attested_pose") or entities[0].get("pose") or {}
        ox = float(origin.get("x", 0))
        oy = float(origin.get("y", 0))
        for ent in entities:
            pose = ent.get("truth_attested_pose") or ent.get("pose") or {}
            try:
                dx = float(pose.get("x", 0)) - ox
                dy = float(pose.get("y", 0)) - oy
                if math.hypot(dx, dy) <= NOMINAL_DOME_RADIUS_M:
                    count += 1
            except (TypeError, ValueError):
                continue
    return {"sensor_id": "radar_north", "entities_in_nominal_dome": count}


def build_fidelity_truth_snapshot(
    *,
    session_id: str,
    timestamp_utc: str,
    entity_truth: list[dict[str, Any]],
    attestation_status: str = "available",
    ground_z_m: float = 0.0,
) -> dict[str, Any]:
    """Build rt_fidelity_truth_snapshot_v1 from sim entity truth rows."""
    return {
        "schema": FIDELITY_SCHEMA,
        "session_id": session_id,
        "timestamp_utc": timestamp_utc,
        "governance_banner": FIDELITY_BANNER,
        "attestation_status": attestation_status,
        "entity_truth": entity_truth,
        "los_truth": _mock_los_label(entity_truth),
        "dome_truth": _mock_dome_truth(entity_truth),
        "ground_z_m": ground_z_m,
    }


def build_entity_truth_rows(
    entities: list[dict[str, Any]],
    *,
    feedback_pose_for: Any,
    ground_z_m: float,
) -> list[dict[str, Any]]:
    """Map worker entity list to entity_truth rows with sim AGL."""
    rows: list[dict[str, Any]] = []
    for ent in entities:
        eid = str(ent.get("entity_id", ""))
        if not eid:
            continue
        if callable(feedback_pose_for):
            pose = feedback_pose_for(eid)
        else:
            pose = dict(ent.get("pose") or {})
        if not pose:
            continue
        try:
            sim_agl_m = float(pose.get("z", 0)) - float(ground_z_m)
        except (TypeError, ValueError):
            sim_agl_m = None
        rows.append(
            {
                "entity_id": eid,
                "entity_type": ent.get("entity_type"),
                "sim_entity_ref": ent.get("sim_entity_ref"),
                "truth_attested_pose": dict(pose),
                "sim_agl_m": sim_agl_m,
            }
        )
    return rows


def fidelity_truth_stale(
    fidelity_truth: dict[str, Any] | None,
    config: GovernanceConfig,
) -> bool:
    if not fidelity_truth:
        return True
    if str(fidelity_truth.get("attestation_status")) == "stale":
        return True
    ts = fidelity_truth.get("timestamp_utc")
    return is_poll_stale(ts, config.fidelity_truth_stale_s)


def detect_fidelity_truth_mismatch(
    fidelity_truth: dict[str, Any] | None,
    registry_entity_ids: set[str],
    *,
    registry_sim_refs: dict[str, str] | None = None,
) -> list[str]:
    """Return entity_ids with truth/registry mismatch (explanatory only)."""
    if not fidelity_truth:
        return []
    mismatches: list[str] = []
    refs = registry_sim_refs or {}
    for row in fidelity_truth.get("entity_truth") or []:
        eid = str(row.get("entity_id", ""))
        if not eid:
            continue
        if eid not in registry_entity_ids:
            mismatches.append(eid)
            continue
        sim_ref = row.get("sim_entity_ref")
        if sim_ref is not None and refs.get(eid) and str(sim_ref) != str(refs[eid]):
            mismatches.append(eid)
    return mismatches


def build_fidelity_poll_audits(
    session: Any,
    config: GovernanceConfig,
) -> list[tuple[str, str, dict[str, Any] | None]]:
    """Return list of (command_type, result, detail) for fidelity poll audits."""
    if not fidelity_coupling_active(session, config):
        return []
    mirror = getattr(session, "telemetry_mirror", None)
    if mirror is None or mirror.fidelity_truth is None:
        return []

    truth = mirror.fidelity_truth
    detail: dict[str, Any] = {
        "attestation_status": truth.get("attestation_status"),
        "telemetry_revision": mirror.telemetry_revision,
        "timestamp_utc": truth.get("timestamp_utc"),
    }
    audits: list[tuple[str, str, dict[str, Any] | None]] = [
        ("fidelity_truth_update", "OK", dict(detail)),
    ]

    world = session.world
    registry_ids: set[str] = set()
    if world is not None:
        for ent in world.registry.all_entities():
            registry_ids.add(ent.entity_id)

    mismatch_ids = detect_fidelity_truth_mismatch(truth, registry_ids)
    if mismatch_ids:
        audits.append(
            (
                "fidelity_truth_mismatch",
                "OK",
                {**detail, "entity_ids": mismatch_ids},
            )
        )

    if fidelity_truth_stale(truth, config):
        audits.append(
            (
                "fidelity_truth_stale",
                "OK",
                {**detail, "reason": "clock_age_or_attestation_stale"},
            )
        )

    sync_health = None
    if session.pose_sync is not None and world is not None:
        sync_health = session.pose_sync.summary(world.revision).get("sync_health")
    if sync_health and sync_health not in {"ok", None}:
        if not any(a[0] == "fidelity_truth_stale" for a in audits):
            audits.append(
                (
                    "fidelity_truth_stale",
                    "OK",
                    {**detail, "reason": f"sync_{sync_health}"},
                )
            )

    return audits


def truth_pose_by_entity(fidelity_truth: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not fidelity_truth:
        return {}
    out: dict[str, dict[str, Any]] = {}
    for row in fidelity_truth.get("entity_truth") or []:
        eid = str(row.get("entity_id", ""))
        pose = row.get("truth_attested_pose")
        if eid and isinstance(pose, dict):
            out[eid] = dict(pose)
    return out


def sim_agl_by_entity(fidelity_truth: dict[str, Any] | None) -> dict[str, float | None]:
    if not fidelity_truth:
        return {}
    out: dict[str, float | None] = {}
    for row in fidelity_truth.get("entity_truth") or []:
        eid = str(row.get("entity_id", ""))
        if eid:
            out[eid] = row.get("sim_agl_m")
    return out


def pose_truth_drift_m(
    command_pose: dict[str, Any] | None,
    truth_pose: dict[str, Any] | None,
) -> float | None:
    if not command_pose or not truth_pose:
        return None
    try:
        return pose_drift_m(command_pose, truth_pose)
    except (KeyError, TypeError, ValueError):
        return None


def build_fidelity_telemetry_fields(
    session: Any,
    config: GovernanceConfig,
) -> dict[str, Any]:
    """Additive pull metadata for world_summary / session_health (PLAT-RT-F5b P1)."""
    if not fidelity_coupling_active(session, config):
        return {
            "enable_fidelity_coupling": False,
            "fidelity_attestation_status": "unavailable",
        }

    mirror = getattr(session, "telemetry_mirror", None)
    truth = mirror.fidelity_truth if mirror is not None else None
    base: dict[str, Any] = {
        "enable_fidelity_coupling": True,
        "fidelity_label": "truth_attested",
        "governance_banner": FIDELITY_BANNER,
    }
    if not isinstance(truth, dict):
        base["fidelity_attestation_status"] = "unavailable"
        return base

    stale = fidelity_truth_stale(truth, config)
    attestation = str(truth.get("attestation_status") or "available")
    if stale or attestation == "stale":
        attestation = "stale"
    elif attestation not in {"available", "stale"}:
        attestation = "available"
    base["fidelity_attestation_status"] = attestation
    base["fidelity_truth"] = dict(truth)
    return base
