"""Gazebo service helpers for RT sandbox bridge (PLAT-RT-G6)."""

from __future__ import annotations

import math
import subprocess


def fmt_remove_model_req(model_name: str) -> str:
    return f'name: "{model_name}", type: MODEL'


def fmt_spawn_model_from_file(
    sdf_absolute_path: str,
    instance_name: str,
    x: float,
    y: float,
    z: float,
) -> str:
    p = sdf_absolute_path.replace('\\', '/')
    return (
        f'sdf_filename: "{p}", '
        f'name: "{instance_name}", '
        f'allow_renaming: true, '
        f'pose {{ '
        f'position {{ x: {x}, y: {y}, z: {z} }}, '
        f'orientation {{ x: 0, y: 0, z: 0, w: 1 }} '
        f'}}'
    )


def fmt_pose_req(name: str, x: float, y: float, z: float, yaw_deg: float = 0.0) -> str:
    half = math.radians(float(yaw_deg)) * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return (
        f'name: "{name}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    )


def gz_service(world_name: str, service: str, req: str, timeout_ms: int = 3000) -> bool:
    if not service.startswith('/'):
        service = f'/world/{world_name}/{service}'
    cmd = [
        'gz',
        'service',
        '-s',
        service,
        '--reqtype',
        _reqtype_for(service),
        '--reptype',
        _reptype_for(service),
        '--timeout',
        str(max(100, timeout_ms)),
        '--req',
        req,
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 2)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def _reqtype_for(service: str) -> str:
    if service.endswith('set_pose'):
        return 'gz.msgs.Pose'
    if service.endswith('create'):
        return 'gz.msgs.EntityFactory'
    if service.endswith('remove'):
        return 'gz.msgs.Entity'
    return 'gz.msgs.Pose'


def _reptype_for(service: str) -> str:
    if service.endswith('set_pose'):
        return 'gz.msgs.Boolean'
    if service.endswith('create'):
        return 'gz.msgs.Boolean'
    if service.endswith('remove'):
        return 'gz.msgs.Boolean'
    return 'gz.msgs.Boolean'
