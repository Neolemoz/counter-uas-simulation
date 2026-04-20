"""Shared protobuf text for gz.msgs.Pose used by ``gz service ... set_pose``."""

from __future__ import annotations

import math


def fmt_pose_req(name: str, x: float, y: float, z: float) -> str:
    return fmt_pose_req_full(name, x, y, z, 0.0, 0.0, 0.0, 1.0)


def fmt_pose_req_full(name: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> str:
    return (
        f'name: "{name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )


def quat_align_body_x_to_velocity(vx: float, vy: float, vz: float) -> tuple[float, float, float, float]:
    """
    Quaternion (x,y,z,w) such that the model's **body +X axis** aligns with ``(vx,vy,vz)``.

    Interceptor meshes are authored with forward = +X; this makes the drone "fly nose-first"
    toward the commanded direction.
    """
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    vx, vy, vz = vx / n, vy / n, vz / n
    ex = (1.0, 0.0, 0.0)
    dot = max(-1.0, min(1.0, vx))
    if dot > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-9:
        # 180°: body +X -> -X ; rotate 180° about Z
        return (0.0, 0.0, 1.0, 0.0)
    cx = ex[1] * vz - ex[2] * vy
    cy = ex[2] * vx - ex[0] * vz
    cz = ex[0] * vy - ex[1] * vx
    alen = math.sqrt(cx * cx + cy * cy + cz * cz)
    if alen < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = cx / alen, cy / alen, cz / alen
    angle = math.acos(dot)
    half = angle * 0.5
    sh = math.sin(half)
    return (ax * sh, ay * sh, az * sh, math.cos(half))
