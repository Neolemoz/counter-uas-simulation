"""Entity type → Gazebo model mapping and ground snap (PLAT-RT-G6)."""

from __future__ import annotations

from typing import Any

ENTITY_GROUND_SNAP_Z: dict[str, float] = {
    "drone": 0.5,
    "radar": 1.0,
    "interceptor": 0.3,
    "waypoint_marker": 0.4,
}


def snap_pose_for_gazebo(
    entity_type: str,
    pose: dict[str, float],
    *,
    enabled: bool = True,
) -> dict[str, float]:
    """Adjust z for model footprint; registry command pose unchanged upstream."""
    out = dict(pose)
    if not enabled:
        return out
    offset = ENTITY_GROUND_SNAP_Z.get(entity_type, 0.0)
    if "z" in out:
        out["z"] = float(out["z"]) + offset
    return out


def unsnap_feedback_pose(
    entity_type: str,
    pose: dict[str, float],
    *,
    enabled: bool = True,
) -> dict[str, float]:
    """Convert Gazebo truth back to registry frame for drift comparison."""
    out = dict(pose)
    if not enabled:
        return out
    offset = ENTITY_GROUND_SNAP_Z.get(entity_type, 0.0)
    if "z" in out:
        out["z"] = float(out["z"]) - offset
    return out


def entity_state_to_feedback(state: dict[str, Any], *, ground_snap_enabled: bool = True) -> dict[str, Any]:
    """Map rt_entity_state_v1 → rt_adapter_feedback_v1."""
    entities = []
    for item in list(state.get("entities") or []):
        entity_type = str(item.get("entity_type", "drone"))
        pose = dict(item.get("pose") or {})
        entities.append(
            {
                "entity_id": item.get("entity_id"),
                "entity_type": entity_type,
                "sim_entity_ref": item.get("sim_entity_ref"),
                "pose": unsnap_feedback_pose(entity_type, pose, enabled=ground_snap_enabled),
            }
        )
    return {
        "schema": "rt_adapter_feedback_v1",
        "timestamp_utc": state.get("timestamp_utc"),
        "sync_seq": int(state.get("sync_seq") or 0),
        "entities": entities,
    }
