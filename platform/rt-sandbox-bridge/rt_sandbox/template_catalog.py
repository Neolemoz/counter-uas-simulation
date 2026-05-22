"""Builtin RT-local runtime templates (PLAT-RT-S6). Not SA scenario corpus."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

TEMPLATE_SCHEMA = "rt_runtime_template_v1"


@dataclass(frozen=True)
class TemplateEntity:
    entity_type: str
    pose: dict[str, float]


@dataclass(frozen=True)
class RuntimeTemplate:
    template_id: str
    kind: str
    description: str
    entities: tuple[TemplateEntity, ...]

    @property
    def entity_count(self) -> int:
        return len(self.entities)


def _e(entity_type: str, x: float, y: float, z: float, yaw: float = 0.0) -> TemplateEntity:
    return TemplateEntity(
        entity_type=entity_type,
        pose={"x": x, "y": y, "z": z, "yaw_deg": yaw},
    )


_BUILTIN_TEMPLATES: tuple[RuntimeTemplate, ...] = (
    RuntimeTemplate(
        template_id="world_empty_v1",
        kind="world_init",
        description="Prototype empty world marker (no entities)",
        entities=(),
    ),
    RuntimeTemplate(
        template_id="radar_north_arc_v1",
        kind="radar_preset",
        description="Two radar placeholders on northern arc",
        entities=(
            _e("radar", -120.0, 280.0, 15.0),
            _e("radar", 120.0, 280.0, 15.0),
        ),
    ),
    RuntimeTemplate(
        template_id="radar_valley_pair_v1",
        kind="radar_preset",
        description="Valley-style radar pair for staging demos",
        entities=(
            _e("radar", -80.0, 120.0, 25.0),
            _e("radar", 80.0, 120.0, 25.0),
        ),
    ),
    RuntimeTemplate(
        template_id="drone_ingress_lane_v1",
        kind="drone_preset",
        description="Single drone on ingress lane (prototype)",
        entities=(_e("drone", 0.0, -200.0, 40.0, 90.0),),
    ),
    RuntimeTemplate(
        template_id="drone_patrol_pair_v1",
        kind="drone_preset",
        description="Two drones for patrol staging",
        entities=(
            _e("drone", -60.0, -150.0, 35.0),
            _e("drone", 60.0, -150.0, 35.0),
        ),
    ),
    RuntimeTemplate(
        template_id="waypoint_patrol_triangle_v1",
        kind="waypoint_layout",
        description="Triangular waypoint marker layout",
        entities=(
            _e("waypoint_marker", 0.0, 100.0, 5.0),
            _e("waypoint_marker", -100.0, -50.0, 5.0),
            _e("waypoint_marker", 100.0, -50.0, 5.0),
        ),
    ),
    RuntimeTemplate(
        template_id="interceptor_ready_pair_v1",
        kind="drone_preset",
        description="Interceptor placeholders at ready positions",
        entities=(
            _e("interceptor", -40.0, 0.0, 10.0),
            _e("interceptor", 40.0, 0.0, 10.0),
        ),
    ),
)

_TEMPLATE_BY_ID: dict[str, RuntimeTemplate] = {t.template_id: t for t in _BUILTIN_TEMPLATES}


def list_template_ids() -> list[str]:
    return sorted(_TEMPLATE_BY_ID.keys())


def get_template(template_id: str) -> RuntimeTemplate | None:
    return _TEMPLATE_BY_ID.get(template_id)


def list_templates_metadata() -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for t in sorted(_BUILTIN_TEMPLATES, key=lambda x: x.template_id):
        out.append(
            {
                "schema": TEMPLATE_SCHEMA,
                "template_id": t.template_id,
                "kind": t.kind,
                "entity_count": t.entity_count,
                "description": t.description,
            }
        )
    return out


def catalog_size() -> int:
    return len(_TEMPLATE_BY_ID)
