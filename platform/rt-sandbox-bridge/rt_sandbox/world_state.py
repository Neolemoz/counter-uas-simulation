"""Transient session world state store (PLAT-RT-S3)."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from rt_sandbox.entity_registry import EntityRecord, EntityRegistry
from rt_sandbox.governance import WORLD_BOUNDS


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class WorldSnapshot:
    schema: str
    session_id: str
    revision: int
    entity_count: int
    entities: list[dict[str, Any]]
    bounds: dict[str, Any]
    timestamp_utc: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "session_id": self.session_id,
            "revision": self.revision,
            "entity_count": self.entity_count,
            "entities": self.entities,
            "bounds": self.bounds,
            "timestamp_utc": self.timestamp_utc,
        }


@dataclass
class WorldStateStore:
    session_id: str
    registry: EntityRegistry = field(init=False)
    revision: int = 0

    def __post_init__(self) -> None:
        self.registry = EntityRegistry(session_id=self.session_id)

    def bump_revision(self) -> int:
        self.revision += 1
        return self.revision

    def snapshot(self) -> WorldSnapshot:
        entities = [
            {
                "entity_id": e.entity_id,
                "entity_type": e.entity_type,
                "pose": dict(e.pose),
            }
            for e in self.registry.all_entities()
        ]
        return WorldSnapshot(
            schema="rt_world_snapshot_v1",
            session_id=self.session_id,
            revision=self.revision,
            entity_count=self.registry.count(),
            entities=entities,
            bounds=dict(WORLD_BOUNDS),
            timestamp_utc=_utc_now(),
        )

    def world_summary(self) -> dict[str, Any]:
        summary = self.registry.summary()
        summary["revision"] = self.revision
        summary["bounds"] = dict(WORLD_BOUNDS)
        return summary

    def reset(self) -> int:
        removed = self.registry.clear()
        self.bump_revision()
        return removed

    def clear(self) -> int:
        return self.reset()
