"""Bridge entity operations → adapter pose sync with feedback mirror (PLAT-RT-G3)."""

from __future__ import annotations

from typing import Any

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.pose_sync import PoseSyncMirror, new_pose_sync_mirror
from rt_sandbox.runtime_handle import RuntimeHandle, runtime_is_adapter


def ensure_pose_sync_mirror(session: Any) -> PoseSyncMirror:
    mirror = getattr(session, "pose_sync", None)
    if mirror is None:
        mirror = new_pose_sync_mirror(session.session_id)
        session.pose_sync = mirror
    return mirror


def clear_pose_sync(session: Any) -> None:
    mirror = getattr(session, "pose_sync", None)
    if mirror is not None:
        mirror.clear()


def sync_spawn(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entity_id: str,
    entity_type: str,
    pose: dict[str, float],
    *,
    bridge_revision: int | None = None,
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return None
    return runtime.apply_pose(
        entity_id,
        entity_type,
        pose,
        bridge_revision=bridge_revision,
    )


def sync_move(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entity_id: str,
    entity_type: str,
    pose: dict[str, float],
    *,
    bridge_revision: int | None = None,
) -> dict[str, Any] | None:
    return sync_spawn(
        runtime,
        config,
        entity_id,
        entity_type,
        pose,
        bridge_revision=bridge_revision,
    )


def sync_delete(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entity_id: str,
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return None
    return runtime.delete_entity(entity_id)


def sync_reset_world(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
) -> None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return
    runtime.reset_world()


def poll_feedback(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    *,
    mock_inject_drift: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not config.pose_sync_enabled:
        return None
    if not runtime_is_adapter(runtime):
        return None
    poll = getattr(runtime, "poll_feedback", None)
    if poll is None:
        return None
    return poll(mock_inject_drift=mock_inject_drift)


def resync_all_poses(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entities: list[dict[str, Any]],
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return None
    fn = getattr(runtime, "resync_all", None)
    if fn is None:
        return None
    return fn(entities)


def sync_audit_event(err_code: str | None, health: str) -> str | None:
    if err_code == "SYNC_STALE":
        return "sync_stale"
    if err_code == "SYNC_MISMATCH":
        return "sync_mismatch"
    if health == "ok":
        return "sync_update"
    return None


def run_post_entity_sync(
    session: Any,
    config: GovernanceConfig,
    *,
    entity_id: str,
    entity_type: str,
    command_pose: dict[str, float],
    sync_revision: int,
    push_result: dict[str, Any] | None,
) -> tuple[dict[str, Any] | None, str | None, str | None]:
    """Poll feedback, update mirror, return (enriched push_result, error_code, audit_event)."""
    if not config.enable_gazebo_adapter or not config.pose_sync_enabled:
        return push_result, None, None
    if not runtime_is_adapter(session.runtime):
        return push_result, None, None

    mirror = ensure_pose_sync_mirror(session)
    sim_ref = None
    if push_result and not push_result.get("error_code"):
        sim_ref = push_result.get("sim_entity_ref")
    mirror.record_command(
        entity_id,
        entity_type,
        command_pose,
        sync_revision,
        sim_entity_ref=str(sim_ref) if sim_ref else None,
    )

    feedback = poll_feedback(session.runtime, config)
    if feedback is None:
        return push_result, None, None
    if feedback.get("error_code"):
        mirror.sync_health = "feedback_lost"
        return push_result, str(feedback["error_code"]), "adapter_feedback_lost"

    world = session.world
    registry_ids = set()
    if world is not None:
        registry_ids = {e.entity_id for e in world.registry.all_entities()}

    health, err_code, audit_detail = mirror.update_from_feedback(
        feedback, registry_ids, config
    )
    enriched = dict(push_result or {})
    enriched["sync_health"] = health
    enriched["sync_revision"] = sync_revision
    if audit_detail:
        enriched["sync_detail"] = audit_detail

    audit_event = sync_audit_event(err_code, health)
    if audit_event == "sync_update" and audit_detail is None and entity_id in mirror.entries:
        ent = mirror.entries[entity_id]
        audit_detail = {
            "entity_id": entity_id,
            "sync_revision": sync_revision,
            "drift_m": ent.drift_m,
            "sim_entity_ref": ent.sim_entity_ref,
        }
    if audit_detail:
        enriched["sync_detail"] = audit_detail

    return enriched, err_code, audit_event
