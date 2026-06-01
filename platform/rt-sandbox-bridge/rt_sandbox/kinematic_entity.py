"""Entity record helpers for kinematic plant integration (mock adapter path)."""

from __future__ import annotations

import time
from typing import Any

from rt_sandbox.kinematic_plant import (
    AeroEnvironment,
    KinematicLimits,
    PlantState,
    integrate_toward_pose,
    is_mobile_entity,
    snap_to_commanded,
    telemetry_from_state,
)


def default_entity_record(entity_type: str, commanded_pose: dict[str, float]) -> dict[str, Any]:
    pose = dict(commanded_pose)
    return {
        "entity_type": entity_type,
        "pose": pose,
        "commanded_pose": dict(commanded_pose),
        "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "last_integrate_monotonic": time.monotonic(),
    }


def advance_entity_toward_command(
    ent: dict[str, Any],
    commanded_pose: dict[str, float],
    *,
    limits: KinematicLimits,
    plant_enabled: bool,
    aero: AeroEnvironment | None = None,
    dt: float | None = None,
    now: float | None = None,
) -> dict[str, float]:
    """Update entity dict in place; return integrated registry-frame pose."""
    entity_type = str(ent.get("entity_type", "drone"))
    commanded = dict(commanded_pose)
    ent["commanded_pose"] = commanded

    if not plant_enabled or not is_mobile_entity(entity_type):
        plant = snap_to_commanded(PlantState.from_pose(ent.get("pose") or {}), commanded)
        ent["pose"] = plant.as_pose()
        ent["velocity"] = {"x": 0.0, "y": 0.0, "z": 0.0}
        ent["last_integrate_monotonic"] = now if now is not None else time.monotonic()
        return dict(ent["pose"])

    t_now = now if now is not None else time.monotonic()
    last = ent.get("last_integrate_monotonic")
    if dt is None:
        dt = float(t_now - float(last)) if last is not None else 0.0
    ent["last_integrate_monotonic"] = t_now

    plant = PlantState.from_pose(ent.get("pose") or {}, ent.get("velocity"))
    if last is None or dt <= 0.0:
        plant = snap_to_commanded(plant, commanded)
    else:
        plant = integrate_toward_pose(plant, commanded, dt, limits, aero)

    ent["pose"] = plant.as_pose()
    ent["velocity"] = {
        "x": plant.vx,
        "y": plant.vy,
        "z": plant.vz,
    }
    return dict(ent["pose"])


def entity_telemetry_fields(ent: dict[str, Any]) -> dict[str, Any]:
    plant = PlantState.from_pose(ent.get("pose") or {}, ent.get("velocity"))
    return telemetry_from_state(plant)
