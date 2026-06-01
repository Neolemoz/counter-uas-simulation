"""Lightweight kinematic integrator for RT sandbox pose commands (Step 2 aero realism)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

MOBILE_ENTITY_TYPES = frozenset({'drone', 'interceptor', 'waypoint_marker'})


@dataclass(frozen=True)
class KinematicLimits:
    max_speed_mps: float = 25.0
    max_accel_mps2: float = 30.0
    max_turn_rate_rad_s: float = 2.5
    max_climb_mps: float = 8.0

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> KinematicLimits:
        if not data:
            return cls()
        return cls(
            max_speed_mps=float(data.get('max_speed_mps', 25.0)),
            max_accel_mps2=float(data.get('max_accel_mps2', 30.0)),
            max_turn_rate_rad_s=float(data.get('max_turn_rate_rad_s', 2.5)),
            max_climb_mps=float(data.get('max_climb_mps', 8.0)),
        )


@dataclass(frozen=True)
class AeroEnvironment:
    """Optional drag and wind (defaults off for backward compatibility)."""

    drag_decel_per_mps: float = 0.0
    wind_x_mps: float = 0.0
    wind_y_mps: float = 0.0
    wind_z_mps: float = 0.0

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> AeroEnvironment:
        if not data:
            return cls()
        return cls(
            drag_decel_per_mps=float(data.get('drag_decel_per_mps', 0.0)),
            wind_x_mps=float(data.get('wind_x_mps', 0.0)),
            wind_y_mps=float(data.get('wind_y_mps', 0.0)),
            wind_z_mps=float(data.get('wind_z_mps', 0.0)),
        )


@dataclass
class PlantState:
    x: float
    y: float
    z: float
    yaw_deg: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    def as_pose(self) -> dict[str, float]:
        return {'x': self.x, 'y': self.y, 'z': self.z, 'yaw_deg': self.yaw_deg}

    @classmethod
    def from_pose(
        cls,
        pose: dict[str, float],
        velocity: dict[str, float] | None = None,
    ) -> PlantState:
        vel = velocity or {}
        return cls(
            x=float(pose.get('x', 0.0)),
            y=float(pose.get('y', 0.0)),
            z=float(pose.get('z', 0.0)),
            yaw_deg=float(pose.get('yaw_deg', 0.0)),
            vx=float(vel.get('x', vel.get('vx', 0.0))),
            vy=float(vel.get('y', vel.get('vy', 0.0))),
            vz=float(vel.get('z', vel.get('vz', 0.0))),
        )


def is_mobile_entity(entity_type: str) -> bool:
    return entity_type in MOBILE_ENTITY_TYPES


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _unit_horiz(x: float, y: float) -> tuple[float, float]:
    n = math.hypot(x, y)
    if n < 1e-12:
        return 1.0, 0.0
    return x / n, y / n


def _limit_horiz_turn(
    vx: float,
    vy: float,
    des_x: float,
    des_y: float,
    max_angle: float,
) -> tuple[float, float]:
    speed = math.hypot(vx, vy)
    cur_x, cur_y = _unit_horiz(vx, vy)
    des_ux, des_uy = _unit_horiz(des_x, des_y)
    dot = _clamp(cur_x * des_ux + cur_y * des_uy, -1.0, 1.0)
    angle = math.acos(dot)
    cross = cur_x * des_uy - cur_y * des_ux
    if angle < 1e-12:
        return vx, vy
    step = min(angle, max_angle)
    if cross < 0.0:
        step = -step
    c = math.cos(step)
    s = math.sin(step)
    nx = cur_x * c - cur_y * s
    ny = cur_x * s + cur_y * c
    if speed < 1e-9:
        return nx * 0.0, ny * 0.0
    return nx * speed, ny * speed


def _apply_linear_drag(
    vx: float,
    vy: float,
    vz: float,
    dt: float,
    drag_decel_per_mps: float,
) -> tuple[float, float, float]:
    """Speed-dependent deceleration: |a| = drag_decel_per_mps * |v|, opposite velocity."""
    if drag_decel_per_mps <= 0.0 or dt <= 0.0:
        return vx, vy, vz
    spd = _norm3(vx, vy, vz)
    if spd < 1e-12:
        return 0.0, 0.0, 0.0
    dv = drag_decel_per_mps * spd * dt
    if dv >= spd:
        return 0.0, 0.0, 0.0
    scale = (spd - dv) / spd
    return vx * scale, vy * scale, vz * scale


def _ground_velocity(
    vx: float,
    vy: float,
    vz: float,
    aero: AeroEnvironment,
) -> tuple[float, float, float]:
    return (
        vx + aero.wind_x_mps,
        vy + aero.wind_y_mps,
        vz + aero.wind_z_mps,
    )


def integrate_toward_pose(
    state: PlantState,
    commanded: dict[str, float],
    dt: float,
    limits: KinematicLimits,
    aero: AeroEnvironment | None = None,
) -> PlantState:
    """Advance plant one step toward commanded pose with speed/accel/turn/climb limits."""
    if aero is None:
        aero = AeroEnvironment()
    if dt <= 0.0:
        return state

    air_vx = state.vx - aero.wind_x_mps
    air_vy = state.vy - aero.wind_y_mps
    air_vz = state.vz - aero.wind_z_mps

    cmd_x = float(commanded.get('x', state.x))
    cmd_y = float(commanded.get('y', state.y))
    cmd_z = float(commanded.get('z', state.z))
    cmd_yaw = float(commanded.get('yaw_deg', state.yaw_deg))

    ex = cmd_x - state.x
    ey = cmd_y - state.y
    ez = cmd_z - state.z
    dist = _norm3(ex, ey, ez)

    if dist < 1e-9:
        v_des = (0.0, 0.0, 0.0)
    else:
        inv = 1.0 / dist
        ux, uy, uz = ex * inv, ey * inv, ez * inv
        s_des = min(limits.max_speed_mps, dist / dt)
        v_des = (ux * s_des, uy * s_des, uz * s_des)

    des_speed_h = math.hypot(v_des[0], v_des[1])
    if des_speed_h > 1e-9:
        turn_vx, turn_vy = _limit_horiz_turn(
            air_vx,
            air_vy,
            v_des[0],
            v_des[1],
            limits.max_turn_rate_rad_s * dt,
        )
        turn_norm = math.hypot(turn_vx, turn_vy)
        if turn_norm > 1e-9:
            scale_h = des_speed_h / turn_norm
            v_des = (turn_vx * scale_h, turn_vy * scale_h, v_des[2])
        else:
            du_x, du_y = _unit_horiz(v_des[0], v_des[1])
            v_des = (du_x * des_speed_h, du_y * des_speed_h, v_des[2])

    dvx = v_des[0] - air_vx
    dvy = v_des[1] - air_vy
    dvz = v_des[2] - air_vz
    dv = _norm3(dvx, dvy, dvz)
    max_dv = limits.max_accel_mps2 * dt
    if dv > max_dv and dv > 1e-12:
        scale = max_dv / dv
        dvx *= scale
        dvy *= scale
        dvz *= scale

    vx = air_vx + dvx
    vy = air_vy + dvy
    vz = air_vz + dvz

    vz = _clamp(vz, -limits.max_climb_mps, limits.max_climb_mps)

    spd = _norm3(vx, vy, vz)
    if spd > limits.max_speed_mps and spd > 1e-12:
        scale = limits.max_speed_mps / spd
        vx *= scale
        vy *= scale
        vz *= scale

    vx, vy, vz = _apply_linear_drag(vx, vy, vz, dt, aero.drag_decel_per_mps)
    gx, gy, gz = _ground_velocity(vx, vy, vz, aero)

    nx = state.x + gx * dt
    ny = state.y + gy * dt
    nz = state.z + gz * dt

    h_spd = math.hypot(gx, gy)
    if h_spd > 0.25:
        yaw_deg = math.degrees(math.atan2(gy, gx))
    else:
        yaw_deg = cmd_yaw

    return PlantState(x=nx, y=ny, z=nz, yaw_deg=yaw_deg, vx=gx, vy=gy, vz=gz)


def snap_to_commanded(state: PlantState, commanded: dict[str, float]) -> PlantState:
    """Instant placement (kinematic plant disabled or static entity)."""
    return PlantState(
        x=float(commanded.get('x', state.x)),
        y=float(commanded.get('y', state.y)),
        z=float(commanded.get('z', state.z)),
        yaw_deg=float(commanded.get('yaw_deg', state.yaw_deg)),
        vx=0.0,
        vy=0.0,
        vz=0.0,
    )


def telemetry_from_state(state: PlantState) -> dict[str, Any]:
    """Report ground-frame velocity (wind + pursuit already folded into ``PlantState``)."""
    vx, vy, vz = state.vx, state.vy, state.vz
    speed = _norm3(vx, vy, vz)
    h_spd = math.hypot(vx, vy)
    if h_spd > 0.25:
        heading_deg = math.degrees(math.atan2(vy, vx))
    else:
        heading_deg = float(state.yaw_deg)
    return {
        'velocity': {
            'x': float(vx),
            'y': float(vy),
            'z': float(vz),
            'speed_mps': float(speed),
        },
        'speed_mps': float(speed),
        'heading_deg': float(heading_deg),
    }
