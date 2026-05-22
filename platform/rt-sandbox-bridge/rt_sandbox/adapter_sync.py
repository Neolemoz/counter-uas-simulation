"""Bridge entity operations → adapter pose/state sync (PLAT-RT-G2 foundation)."""

from __future__ import annotations

from typing import Any

from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.runtime_handle import RuntimeHandle, runtime_is_adapter


def sync_spawn(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entity_id: str,
    entity_type: str,
    pose: dict[str, float],
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return None
    return runtime.apply_pose(entity_id, entity_type, pose)


def sync_move(
    runtime: RuntimeHandle,
    config: GovernanceConfig,
    entity_id: str,
    entity_type: str,
    pose: dict[str, float],
) -> dict[str, Any] | None:
    if not config.enable_gazebo_adapter or not runtime_is_adapter(runtime):
        return None
    return runtime.apply_pose(entity_id, entity_type, pose)


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
