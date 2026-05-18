"""Shared kinematic plant propagation for live, MC, and offline rollout.

The model is intentionally not aerodynamic: it is a deterministic point-mass
velocity-command plant with command freshness, optional command delay, optional
velocity smoothing/bandwidth, optional turn-rate and acceleration limits, and
Euler position propagation.  Gazebo remains a pose sink for the live path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

Vec3 = tuple[float, float, float]


def norm3(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def clamp_speed(v: Vec3, max_speed_m_s: float) -> Vec3:
    n = norm3(v)
    vmax = max(float(max_speed_m_s), 0.0)
    if n < 1e-9 or n <= vmax or vmax <= 0.0:
        return v if vmax > 0.0 else (0.0, 0.0, 0.0)
    s = vmax / n
    return (v[0] * s, v[1] * s, v[2] * s)


def _unit(v: Vec3) -> Vec3:
    n = norm3(v)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def rotate_dir_toward(a: Vec3, b: Vec3, max_angle_rad: float) -> Vec3:
    ax, ay, az = _unit(a)
    bx, by, bz = _unit(b)
    if norm3((ax, ay, az)) < 1e-9:
        return (bx, by, bz)
    if norm3((bx, by, bz)) < 1e-9:
        return (ax, ay, az)
    dot = max(-1.0, min(1.0, ax * bx + ay * by + az * bz))
    angle = math.acos(dot)
    if angle <= max_angle_rad or angle < 1e-9:
        return (bx, by, bz)
    sin_a = math.sin(angle)
    if sin_a < 1e-9:
        return (bx, by, bz)
    t = max_angle_rad / angle
    fa = math.sin((1.0 - t) * angle) / sin_a
    fb = math.sin(t * angle) / sin_a
    out = (fa * ax + fb * bx, fa * ay + fb * by, fa * az + fb * bz)
    return _unit(out)


def limit_velocity_delta(previous: Vec3, desired: Vec3, *, max_turn_rate_rad_s: float, max_accel_m_s2: float, dt_s: float) -> Vec3:
    """Apply turn-rate limiting first, then acceleration limiting."""
    dt = max(float(dt_s), 1e-6)
    nx, ny, nz = desired
    if max_turn_rate_rad_s > 1e-9:
        p_spd = norm3(previous)
        d_spd = norm3(desired)
        if p_spd > 1e-6 and d_spd > 1e-6:
            u = rotate_dir_toward(previous, desired, max_turn_rate_rad_s * dt)
            nx, ny, nz = u[0] * d_spd, u[1] * d_spd, u[2] * d_spd

    dv = (nx - previous[0], ny - previous[1], nz - previous[2])
    dvn = norm3(dv)
    max_dv = max(0.0, float(max_accel_m_s2)) * dt
    if max_dv > 1e-12 and dvn > max_dv and dvn > 1e-9:
        s = max_dv / dvn
        dv = (dv[0] * s, dv[1] * s, dv[2] * s)
    return (previous[0] + dv[0], previous[1] + dv[1], previous[2] + dv[2])


@dataclass(frozen=True)
class KinematicPlantParams:
    dt_s: float
    max_speed_m_s: float
    max_accel_m_s2: float = 0.0
    max_turn_rate_rad_s: float = 0.0
    autopilot_tau_s: float = 0.0
    use_legacy_ema: bool = False
    legacy_ema_alpha: float = 1.0
    cmd_delay_s: float = 0.0
    cmd_timeout_s: float = 0.75
    fallback_velocity: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class KinematicPlantState:
    position: Vec3
    applied_velocity: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class KinematicPlantMemory:
    smoothed_velocity: Vec3 = (0.0, 0.0, 0.0)
    previous_velocity: Vec3 = (0.0, 0.0, 0.0)
    command_buffer: tuple[Vec3, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class PlantCommand:
    velocity: Vec3
    has_command: bool = True
    age_s: float = 0.0
    idle: bool = False


@dataclass(frozen=True)
class PlantStepResult:
    state: KinematicPlantState
    memory: KinematicPlantMemory
    selected_command: Vec3
    delayed_command: Vec3
    source: str


def command_delay_samples(params: KinematicPlantParams) -> int:
    if params.cmd_delay_s <= 0.0:
        return 0
    return int(math.ceil(params.cmd_delay_s / max(params.dt_s, 1e-6)))


def reset_plant_memory(params: KinematicPlantParams | None = None) -> KinematicPlantMemory:
    if params is None:
        return KinematicPlantMemory()
    n = command_delay_samples(params)
    return KinematicPlantMemory(command_buffer=tuple((0.0, 0.0, 0.0) for _ in range(n)))


def _normalized_buffer(buffer: tuple[Vec3, ...], n: int) -> tuple[Vec3, ...]:
    if n <= 0:
        return tuple()
    if len(buffer) == n:
        return buffer
    if len(buffer) > n:
        return buffer[-n:]
    return tuple((0.0, 0.0, 0.0) for _ in range(n - len(buffer))) + buffer


def _delay_command(command: Vec3, memory: KinematicPlantMemory, params: KinematicPlantParams) -> tuple[Vec3, tuple[Vec3, ...]]:
    n = command_delay_samples(params)
    if n <= 0:
        return command, tuple()
    buf = _normalized_buffer(memory.command_buffer, n)
    new_buf = (*buf, command)
    return new_buf[0], new_buf[1:]


def _smooth_command(command: Vec3, memory: KinematicPlantMemory, params: KinematicPlantParams) -> Vec3:
    if params.autopilot_tau_s > 0.0:
        a = max(params.dt_s, 1e-6) / (max(params.dt_s, 1e-6) + params.autopilot_tau_s)
    elif params.use_legacy_ema:
        a = min(1.0, max(0.0, params.legacy_ema_alpha))
    else:
        return command
    prev = memory.smoothed_velocity
    return (
        prev[0] + a * (command[0] - prev[0]),
        prev[1] + a * (command[1] - prev[1]),
        prev[2] + a * (command[2] - prev[2]),
    )


def step_kinematic_plant(
    state: KinematicPlantState,
    memory: KinematicPlantMemory,
    command: PlantCommand,
    params: KinematicPlantParams,
) -> PlantStepResult:
    dt = max(float(params.dt_s), 1e-6)
    if command.idle:
        reset = reset_plant_memory(params)
        return PlantStepResult(
            state=KinematicPlantState(position=state.position, applied_velocity=(0.0, 0.0, 0.0)),
            memory=reset,
            selected_command=(0.0, 0.0, 0.0),
            delayed_command=(0.0, 0.0, 0.0),
            source="idle",
        )

    if command.has_command and command.age_s <= params.cmd_timeout_s:
        selected = command.velocity
        source = "cmd"
    else:
        selected = params.fallback_velocity
        source = "fallback"
    selected = clamp_speed(selected, params.max_speed_m_s)
    delayed, next_buf = _delay_command(selected, memory, params)
    smoothed = _smooth_command(delayed, memory, params)
    limited = limit_velocity_delta(
        memory.previous_velocity,
        smoothed,
        max_turn_rate_rad_s=params.max_turn_rate_rad_s,
        max_accel_m_s2=params.max_accel_m_s2,
        dt_s=dt,
    )
    limited = clamp_speed(limited, params.max_speed_m_s)
    pos = (
        state.position[0] + limited[0] * dt,
        state.position[1] + limited[1] * dt,
        state.position[2] + limited[2] * dt,
    )
    next_memory = KinematicPlantMemory(
        smoothed_velocity=smoothed,
        previous_velocity=limited,
        command_buffer=next_buf,
    )
    return PlantStepResult(
        state=KinematicPlantState(position=pos, applied_velocity=limited),
        memory=next_memory,
        selected_command=selected,
        delayed_command=delayed,
        source=source,
    )


__all__ = [
    "Vec3",
    "KinematicPlantParams",
    "KinematicPlantState",
    "KinematicPlantMemory",
    "PlantCommand",
    "PlantStepResult",
    "clamp_speed",
    "command_delay_samples",
    "limit_velocity_delta",
    "reset_plant_memory",
    "rotate_dir_toward",
    "step_kinematic_plant",
]
