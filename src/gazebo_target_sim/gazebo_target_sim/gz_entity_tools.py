"""Protobuf text for gz user-command services (remove model, spawn EntityFactory)."""

from __future__ import annotations


def fmt_remove_model_req(model_name: str) -> str:
    return f'name: "{model_name}", type: MODEL'


def fmt_spawn_model_from_file(
    sdf_absolute_path: str,
    instance_name: str,
    x: float,
    y: float,
    z: float,
    allow_renaming: bool = True,
) -> str:
    # Path must exist on the machine running gz sim (usually share install path).
    p = sdf_absolute_path.replace('\\', '/')
    allow = 'true' if allow_renaming else 'false'
    return (
        f'sdf_filename: "{p}", '
        f'name: "{instance_name}", '
        f'allow_renaming: {allow}, '
        f'pose {{ '
        f'position {{ x: {x}, y: {y}, z: {z} }}, '
        f'orientation {{ x: 0, y: 0, z: 0, w: 1 }} '
        f'}}'
    )
