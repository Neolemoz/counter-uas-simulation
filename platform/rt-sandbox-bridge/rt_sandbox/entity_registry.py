"""Transient session-scoped entity registry (PLAT-RT-S3)."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from rt_sandbox.governance import (
    ENTITY_CATALOG,
    ENTITY_TYPE_LIMITS,
    MAX_ENTITY_COUNT,
    validate_pose,
)


@dataclass
class EntityRecord:
    entity_id: str
    entity_type: str
    pose: dict[str, float]
    session_id: str
    created_monotonic: float


@dataclass
class EntityRegistry:
    session_id: str
    max_entity_count: int = MAX_ENTITY_COUNT
    _entities: dict[str, EntityRecord] = field(default_factory=dict)

    def count(self) -> int:
        return len(self._entities)

    def count_by_type(self, entity_type: str) -> int:
        return sum(1 for e in self._entities.values() if e.entity_type == entity_type)

    def all_entities(self) -> list[EntityRecord]:
        return list(self._entities.values())

    def get(self, entity_id: str) -> EntityRecord | None:
        return self._entities.get(entity_id)

    def spawn(
        self,
        entity_type: str,
        pose: dict[str, float],
        *,
        entity_id: str | None = None,
    ) -> tuple[EntityRecord | None, str | None]:
        """Return (record, error_code)."""
        if entity_type not in ENTITY_CATALOG:
            return None, "COMMAND_FORBIDDEN"
        if self.count() >= self.max_entity_count:
            return None, "RESOURCE_LIMIT_EXCEEDED"
        type_limit = ENTITY_TYPE_LIMITS.get(entity_type, 0)
        if self.count_by_type(entity_type) >= type_limit:
            return None, "RESOURCE_LIMIT_EXCEEDED"
        pose_err = validate_pose(pose)
        if pose_err:
            return pose_err
        eid = entity_id or str(uuid.uuid4())
        if eid in self._entities:
            return None, "INVALID_STATE"
        record = EntityRecord(
            entity_id=eid,
            entity_type=entity_type,
            pose=dict(pose),
            session_id=self.session_id,
            created_monotonic=time.monotonic(),
        )
        self._entities[eid] = record
        return record, None

    def move(self, entity_id: str, pose: dict[str, float]) -> tuple[EntityRecord | None, str | None]:
        record = self._entities.get(entity_id)
        if record is None:
            return None, "ENTITY_NOT_FOUND"
        pose_err = validate_pose(pose)
        if pose_err:
            return pose_err
        record.pose = dict(pose)
        return record, None

    def delete(self, entity_id: str) -> tuple[EntityRecord | None, str | None]:
        record = self._entities.pop(entity_id, None)
        if record is None:
            return None, "ENTITY_NOT_FOUND"
        return record, None

    def clear(self) -> int:
        n = len(self._entities)
        self._entities.clear()
        return n

    def summary(self) -> dict[str, Any]:
        by_type: dict[str, int] = {t: 0 for t in ENTITY_CATALOG}
        for e in self._entities.values():
            by_type[e.entity_type] = by_type.get(e.entity_type, 0) + 1
        return {
            "entity_count": self.count(),
            "by_type": by_type,
        }

    def poses_for_telemetry(self, limit: int = 32) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for e in self._entities.values():
            if len(out) >= limit:
                break
            out.append(
                {
                    "entity_id": e.entity_id,
                    "entity_type": e.entity_type,
                    "pose": dict(e.pose),
                }
            )
        return out
