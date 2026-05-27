"""Unit tests for entity model mapping (PLAT-RT-G6)."""

from __future__ import annotations

from rt_sandbox.entity_model_map import (
    entity_state_to_feedback,
    snap_pose_for_gazebo,
    unsnap_feedback_pose,
)


def test_snap_pose_for_gazebo_drone() -> None:
    pose = snap_pose_for_gazebo("drone", {"x": 1, "y": 2, "z": 10})
    assert pose["z"] == 10.5


def test_unsnap_roundtrip() -> None:
    cmd = {"x": 0, "y": 0, "z": 20, "yaw_deg": 0}
    gz = snap_pose_for_gazebo("radar", cmd)
    back = unsnap_feedback_pose("radar", gz)
    assert back["z"] == cmd["z"]


def test_entity_state_to_feedback() -> None:
    state = {
        "schema": "rt_entity_state_v1",
        "timestamp_utc": "2026-05-25T12:00:00+00:00",
        "sync_seq": 2,
        "entities": [
            {
                "entity_id": "e1",
                "entity_type": "drone",
                "sim_entity_ref": "sim-abc",
                "pose": {"x": 1, "y": 2, "z": 10.5},
            }
        ],
    }
    fb = entity_state_to_feedback(state)
    assert fb["schema"] == "rt_adapter_feedback_v1"
    assert fb["entities"][0]["pose"]["z"] == 10.0
