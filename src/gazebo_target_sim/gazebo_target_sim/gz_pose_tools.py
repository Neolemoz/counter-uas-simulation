"""Shared protobuf text for gz.msgs.Pose used by ``gz service ... set_pose``."""

from __future__ import annotations


def fmt_pose_req(name: str, x: float, y: float, z: float) -> str:
    return (
        f'name: "{name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        'orientation: {x: 0, y: 0, z: 0, w: 1}'
    )
