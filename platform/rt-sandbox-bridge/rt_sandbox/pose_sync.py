"""Transient pose sync mirror and stale detection (PLAT-RT-G3).

sync_revision / sync_seq — see docs/evaluation/rt_revision_vocabulary_v1.md
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Literal

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.time_utils import is_poll_stale

SyncHealth = Literal["ok", "stale", "mismatch", "feedback_lost"]


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def pose_drift_m(command: dict[str, float], feedback: dict[str, float]) -> float:
    keys = ("x", "y", "z")
    return math.sqrt(
        sum((float(command[k]) - float(feedback[k])) ** 2 for k in keys)
    )


def pose_yaw_drift_deg(command: dict[str, float], feedback: dict[str, float]) -> float:
    cy = float(command.get("yaw_deg", 0.0))
    fy = float(feedback.get("yaw_deg", 0.0))
    diff = abs(cy - fy) % 360.0
    return min(diff, 360.0 - diff)


@dataclass
class EntitySyncEntry:
    entity_id: str
    sim_entity_ref: str | None = None
    command_pose: dict[str, float] = field(default_factory=dict)
    feedback_pose: dict[str, float] | None = None
    feedback_timestamp_utc: str | None = None
    sync_revision: int = 0
    drift_m: float | None = None

    def to_feedback_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "entity_id": self.entity_id,
            "sim_entity_ref": self.sim_entity_ref,
            "command_pose": dict(self.command_pose),
            "feedback_pose": dict(self.feedback_pose) if self.feedback_pose else None,
            "sync_revision": self.sync_revision,
        }
        if self.drift_m is not None:
            out["drift_m"] = round(self.drift_m, 4)
        return out


@dataclass
class PoseSyncMirror:
    session_id: str
    entries: dict[str, EntitySyncEntry] = field(default_factory=dict)
    sync_health: SyncHealth = "ok"
    last_poll_utc: str | None = None
    last_sync_seq: int = 0
    last_command_utc: str | None = None
    last_feedback_utc: str | None = None
    apply_lag_ms: float | None = None

    def clear(self) -> None:
        self.entries.clear()
        self.sync_health = "ok"
        self.last_poll_utc = None
        self.last_sync_seq = 0
        self.last_command_utc = None
        self.last_feedback_utc = None
        self.apply_lag_ms = None

    def record_command(
        self,
        entity_id: str,
        entity_type: str,
        pose: dict[str, float],
        sync_revision: int,
        sim_entity_ref: str | None = None,
    ) -> None:
        ent = self.entries.get(entity_id)
        if ent is None:
            ent = EntitySyncEntry(entity_id=entity_id)
            self.entries[entity_id] = ent
        ent.command_pose = dict(pose)
        ent.sync_revision = sync_revision
        if sim_entity_ref:
            ent.sim_entity_ref = sim_entity_ref
        self.last_command_utc = _utc_now()

    def remove_entity(self, entity_id: str) -> None:
        self.entries.pop(entity_id, None)

    def update_from_feedback(
        self,
        feedback: dict[str, Any],
        registry_entity_ids: set[str],
        config: GovernanceConfig,
    ) -> tuple[SyncHealth, str | None, dict[str, Any] | None]:
        """Populate mirror from rt_adapter_feedback_v1. Returns health, error_code, audit detail."""
        self.last_poll_utc = str(feedback.get("timestamp_utc") or _utc_now())
        self.last_sync_seq = int(feedback.get("sync_seq") or 0)
        fb_entities = list(feedback.get("entities") or [])
        worst: SyncHealth = "ok"
        audit_detail: dict[str, Any] | None = None
        error_code: str | None = None

        seen_ids: set[str] = set()
        for item in fb_entities:
            eid = str(item.get("entity_id", ""))
            if not eid:
                continue
            seen_ids.add(eid)
            pose = dict(item.get("pose") or {})
            sim_ref = item.get("sim_entity_ref")
            ent = self.entries.get(eid)
            if ent is None and eid in registry_entity_ids:
                ent = EntitySyncEntry(entity_id=eid)
                self.entries[eid] = ent
            if ent is None:
                self.sync_health = "mismatch"
                worst = "mismatch"
                audit_detail = {
                    "entity_id": eid,
                    "reason": "unknown_feedback_entity",
                }
                error_code = "SYNC_MISMATCH"
                continue
            if sim_ref and ent.sim_entity_ref and str(sim_ref) != ent.sim_entity_ref:
                self.sync_health = "mismatch"
                worst = "mismatch"
                audit_detail = {
                    "entity_id": eid,
                    "expected_ref": ent.sim_entity_ref,
                    "actual_ref": sim_ref,
                }
                error_code = "SYNC_MISMATCH"
            ent.feedback_pose = pose
            ent.feedback_timestamp_utc = self.last_poll_utc
            if sim_ref:
                ent.sim_entity_ref = str(sim_ref)
            if ent.command_pose and pose:
                ent.drift_m = pose_drift_m(ent.command_pose, pose)
                yaw_thresh = float(config.pose_sync_yaw_threshold_deg)
                yaw_drift = pose_yaw_drift_deg(ent.command_pose, pose) if yaw_thresh > 0 else 0.0
                pos_stale = ent.drift_m > config.pose_sync_drift_threshold_m
                yaw_stale = yaw_thresh > 0 and yaw_drift > yaw_thresh
                if pos_stale or yaw_stale:
                    self.sync_health = "stale"
                    if worst == "ok":
                        worst = "stale"
                        audit_detail = {
                            "entity_id": eid,
                            "threshold_m": config.pose_sync_drift_threshold_m,
                            "command_pose": dict(ent.command_pose),
                            "feedback_pose": dict(pose),
                            "drift_m": ent.drift_m,
                        }
                        if yaw_stale:
                            audit_detail["yaw_drift_deg"] = yaw_drift
                            audit_detail["yaw_threshold_deg"] = yaw_thresh
                        error_code = "SYNC_STALE"

        for eid in registry_entity_ids:
            if eid not in seen_ids and eid in self.entries:
                ent = self.entries[eid]
                if ent.command_pose:
                    self.sync_health = "mismatch"
                    if worst in {"ok", "stale"}:
                        worst = "mismatch"
                        audit_detail = {
                            "entity_id": eid,
                            "reason": "missing_feedback_entity",
                        }
                        error_code = "SYNC_MISMATCH"

        self.last_feedback_utc = self.last_poll_utc
        if self.last_command_utc and self.last_feedback_utc:
            try:
                cmd_t = datetime.fromisoformat(self.last_command_utc)
                fb_t = datetime.fromisoformat(self.last_feedback_utc)
                self.apply_lag_ms = max(0.0, (fb_t - cmd_t).total_seconds() * 1000.0)
            except ValueError:
                self.apply_lag_ms = None

        self.sync_health = worst
        return worst, error_code, audit_detail

    def check_feedback_stale(self, config: GovernanceConfig) -> bool:
        return is_poll_stale(self.last_poll_utc, config.adapter_feedback_stale_s)

    def check_stale(self, config: GovernanceConfig) -> bool:
        return self.check_feedback_stale(config)

    def summary(
        self,
        sync_revision: int,
        *,
        adapter_mode: str | None = None,
    ) -> dict[str, Any]:
        # sync_revision param is live world.revision — not per-entity mirror revision.
        out: dict[str, Any] = {
            "sync_health": self.sync_health,
            "sync_revision": sync_revision,
            "last_poll_utc": self.last_poll_utc,
            "last_sync_seq": self.last_sync_seq,
            "feedback_entities": [
                e.to_feedback_dict() for e in self.entries.values()
            ],
        }
        if adapter_mode is not None:
            out["adapter_mode"] = adapter_mode
        if self.last_command_utc:
            out["last_command_utc"] = self.last_command_utc
        if self.last_feedback_utc:
            out["last_feedback_utc"] = self.last_feedback_utc
        if self.apply_lag_ms is not None:
            out["apply_lag_ms"] = round(self.apply_lag_ms, 2)
        return out


def new_pose_sync_mirror(session_id: str) -> PoseSyncMirror:
    return PoseSyncMirror(session_id=session_id)
