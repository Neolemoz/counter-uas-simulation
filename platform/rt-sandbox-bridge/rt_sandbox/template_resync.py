"""Template/workflow adapter resync policy (PLAT-RT-R2d).

Policy: docs/evaluation/rt_template_resync_policy_v1.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from rt_sandbox.adapter_poll import adapter_poll_active, run_adapter_poll_tick
from rt_sandbox.adapter_sync import (
    clear_pose_sync,
    resync_all_poses,
    sync_reset_world,
)
from rt_sandbox.governance import GovernanceConfig


@dataclass
class TemplateResyncAudit:
    command_type: str
    result: str
    detail: dict[str, Any] | None = None


@dataclass
class TemplateResyncResult:
    audits: list[TemplateResyncAudit] = field(default_factory=list)
    should_publish_channels: bool = False
    sync_error_code: str | None = None
    resync_ipc_result: dict[str, Any] | None = None
    entity_count: int = 0


def _base_detail(
    *,
    trigger: str,
    template_id: str | None,
    manual: bool,
) -> dict[str, Any]:
    detail: dict[str, Any] = {"trigger": trigger}
    if template_id is not None:
        detail["template_id"] = template_id
    if manual:
        detail["manual"] = True
    return detail


def run_template_adapter_resync(
    session: Any,
    config: GovernanceConfig,
    *,
    trigger: str,
    template_id: str | None = None,
    entities_spawned: int = 0,
    world_reset: bool = False,
    manual: bool = False,
) -> TemplateResyncResult:
    """Apply template resync policy; returns audit entries and channel publish hint."""
    result = TemplateResyncResult()
    base = _base_detail(trigger=trigger, template_id=template_id, manual=manual)

    if not adapter_poll_active(session, config):
        result.audits.append(
            TemplateResyncAudit(
                "template_resync_skipped",
                "OK",
                {**base, "reason": "adapter_inactive"},
            )
        )
        return result

    world = session.world
    if world is None:
        result.audits.append(
            TemplateResyncAudit(
                "template_resync_skipped",
                "OK",
                {**base, "reason": "world_not_initialized"},
            )
        )
        return result

    if not world_reset and entities_spawned == 0 and not manual:
        result.audits.append(
            TemplateResyncAudit(
                "template_resync_skipped",
                "OK",
                {**base, "reason": "no_entities"},
            )
        )
        return result

    entities = [
        {
            "entity_id": e.entity_id,
            "entity_type": e.entity_type,
            "pose": dict(e.pose),
        }
        for e in world.registry.all_entities()
    ]
    result.entity_count = len(entities)

    requested_detail: dict[str, Any] = {
        **base,
        "entity_count": result.entity_count,
        "world_reset": world_reset,
    }
    result.audits.append(
        TemplateResyncAudit("template_resync_requested", "OK", requested_detail)
    )

    if world_reset:
        sync_reset_world(session.runtime, config)
        clear_pose_sync(session)
        poll_result = run_adapter_poll_tick(
            session,
            config,
            poll_feedback=False,
            poll_telemetry=True,
        )
    else:
        ipc_result = resync_all_poses(session.runtime, config, entities)
        result.resync_ipc_result = dict(ipc_result) if ipc_result else None
        if ipc_result and ipc_result.get("error_code"):
            err = str(ipc_result["error_code"])
            result.sync_error_code = err
            result.audits.append(
                TemplateResyncAudit(
                    "template_resync_stale",
                    err,
                    {
                        **base,
                        "entity_count": result.entity_count,
                        "error_code": err,
                        **ipc_result,
                    },
                )
            )
            return result
        poll_result = run_adapter_poll_tick(
            session,
            config,
            poll_feedback=True,
            poll_telemetry=True,
            emit_feedback_audit=False,
        )

    result.should_publish_channels = poll_result.should_publish_channels
    if poll_result.sync_error_code:
        result.sync_error_code = poll_result.sync_error_code
        stale_detail: dict[str, Any] = {
            **base,
            "entity_count": result.entity_count,
            "world_reset": world_reset,
            "sync_error_code": poll_result.sync_error_code,
        }
        if poll_result.audits:
            stale_detail["poll_audit"] = poll_result.audits[-1].command_type
        result.audits.append(
            TemplateResyncAudit("template_resync_stale", poll_result.sync_error_code, stale_detail)
        )
        return result

    completed_detail: dict[str, Any] = {
        **base,
        "entity_count": result.entity_count,
        "world_reset": world_reset,
    }
    if result.resync_ipc_result:
        completed_detail.update(result.resync_ipc_result)
    result.audits.append(
        TemplateResyncAudit("template_resync_completed", "OK", completed_detail)
    )
    return result
