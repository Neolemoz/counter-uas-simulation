"""Apply RT-local runtime templates to session world (PLAT-RT-S6)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from rt_sandbox.governance import FORBIDDEN_SUBSTRINGS
from rt_sandbox.isolation import assert_template_ref_blocked
from rt_sandbox.template_catalog import get_template
from rt_sandbox.world_state import WorldStateStore


@dataclass
class TemplateApplyResult:
    template_id: str
    entities_spawned: int
    entity_ids: list[str]
    revision: int | None


def validate_template_payload(payload: Any) -> str | None:
    if not isinstance(payload, dict):
        return "INVALID_STATE"
    template_id = payload.get("template_id")
    if not isinstance(template_id, str) or not template_id:
        return "INVALID_STATE"
    low = template_id.lower()
    for sub in FORBIDDEN_SUBSTRINGS:
        if sub in low:
            return "COMMAND_FORBIDDEN"
    if "scenario_pack" in low or "fixtures/" in low:
        return "COMMAND_FORBIDDEN"
    try:
        assert_template_ref_blocked(template_id)
    except PermissionError:
        return "COMMAND_FORBIDDEN"
    if get_template(template_id) is None:
        return "INVALID_STATE"
    return None


def apply_template(
    world: WorldStateStore,
    template_id: str,
    *,
    max_entities_per_apply: int,
) -> tuple[TemplateApplyResult | None, str | None]:
    """Return (result, error_code)."""
    tmpl = get_template(template_id)
    if tmpl is None:
        return None, "INVALID_STATE"
    if tmpl.entity_count > max_entities_per_apply:
        return None, "RESOURCE_LIMIT_EXCEEDED"
    if tmpl.entity_count == 0:
        return (
            TemplateApplyResult(
                template_id=template_id,
                entities_spawned=0,
                entity_ids=[],
                revision=None,
            ),
            None,
        )
    registry = world.registry
    entity_ids: list[str] = []
    for ent in tmpl.entities:
        record, err = registry.spawn(ent.entity_type, ent.pose)
        if err:
            return None, err
        assert record is not None
        entity_ids.append(record.entity_id)
    revision = world.bump_revision() if entity_ids else world.revision
    return (
        TemplateApplyResult(
            template_id=template_id,
            entities_spawned=len(entity_ids),
            entity_ids=entity_ids,
            revision=revision,
        ),
        None,
    )
