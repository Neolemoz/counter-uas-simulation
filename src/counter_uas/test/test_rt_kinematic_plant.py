"""Unit tests for RT sandbox kinematic plant (Step 2 aero realism)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_GZ_PKG = _REPO / "src" / "rt_sandbox_gz"
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
for path in (_GZ_PKG, _BRIDGE_PKG):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rt_sandbox.kinematic_entity import advance_entity_toward_command, default_entity_record  # noqa: E402
from rt_sandbox.kinematic_plant import (  # noqa: E402
    AeroEnvironment,
    KinematicLimits,
    PlantState,
    integrate_toward_pose,
    snap_to_commanded,
    telemetry_from_state,
)
from rt_sandbox_gz.kinematic_plant import is_mobile_entity  # noqa: E402


def _limits() -> KinematicLimits:
    return KinematicLimits(
        max_speed_mps=25.0,
        max_accel_mps2=10.0,
        max_turn_rate_rad_s=0.5,
        max_climb_mps=3.0,
    )


def test_accel_ramp_from_rest() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0)
    commanded = {"x": 100.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    lim = _limits()
    dt = 0.1
    out = integrate_toward_pose(state, commanded, dt, lim)
    speed = math.hypot(out.vx, out.vy, out.vz)
    assert speed <= lim.max_accel_mps2 * dt + 1e-6
    assert speed > 0.5


def test_turn_rate_limits_heading_change() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0, vx=10.0, vy=0.0, vz=0.0)
    commanded = {"x": 0.0, "y": 100.0, "z": 0.0, "yaw_deg": 90.0}
    lim = KinematicLimits(
        max_speed_mps=25.0,
        max_accel_mps2=50.0,
        max_turn_rate_rad_s=0.2,
        max_climb_mps=8.0,
    )
    dt = 0.1
    max_step_deg = math.degrees(lim.max_turn_rate_rad_s * dt) + 0.5
    prev_heading = 0.0
    for _ in range(6):
        state = integrate_toward_pose(state, commanded, dt, lim)
        heading = math.degrees(math.atan2(state.vy, state.vx))
        delta = abs(heading - prev_heading)
        if delta > 180.0:
            delta = 360.0 - delta
        assert delta <= max_step_deg
        prev_heading = heading


def test_climb_rate_cap() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0)
    commanded = {"x": 0.0, "y": 0.0, "z": 200.0, "yaw_deg": 0.0}
    lim = KinematicLimits(max_climb_mps=2.0, max_accel_mps2=100.0, max_speed_mps=50.0)
    out = integrate_toward_pose(state, commanded, 0.1, lim)
    assert abs(out.vz) <= lim.max_climb_mps + 1e-6


def test_telemetry_non_zero_velocity() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0, vx=4.0, vy=3.0, vz=0.0)
    telem = telemetry_from_state(state)
    assert telem["speed_mps"] == pytest.approx(5.0)
    assert telem["velocity"]["speed_mps"] == pytest.approx(5.0)
    assert telem["heading_deg"] == pytest.approx(36.8698976, rel=1e-4)


def test_snap_backward_compat_plant_disabled() -> None:
    state = PlantState(x=1.0, y=2.0, z=3.0, yaw_deg=10.0, vx=5.0, vy=0.0, vz=0.0)
    commanded = {"x": 9.0, "y": 8.0, "z": 7.0, "yaw_deg": 45.0}
    out = snap_to_commanded(state, commanded)
    assert out.x == 9.0 and out.y == 8.0 and out.z == 7.0
    assert out.vx == 0.0 and out.vy == 0.0 and out.vz == 0.0


def test_entity_record_integrates_in_mock_path() -> None:
    ent = default_entity_record("drone", {"x": 0.0, "y": 0.0, "z": 10.0, "yaw_deg": 0.0})
    lim = KinematicLimits(max_accel_mps2=5.0, max_speed_mps=25.0)
    advance_entity_toward_command(
        ent,
        {"x": 50.0, "y": 0.0, "z": 10.0, "yaw_deg": 0.0},
        limits=lim,
        plant_enabled=True,
        dt=0.2,
    )
    assert ent["pose"]["x"] > 0.0
    assert ent["pose"]["x"] < 50.0
    assert ent["velocity"]["x"] > 0.0


def test_radar_not_mobile() -> None:
    assert not is_mobile_entity("radar")
    assert is_mobile_entity("interceptor")


def test_plant_disabled_teleports_entity_record() -> None:
    ent = default_entity_record("drone", {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0})
    advance_entity_toward_command(
        ent,
        {"x": 12.0, "y": 3.0, "z": 4.0, "yaw_deg": 15.0},
        limits=_limits(),
        plant_enabled=False,
        dt=1.0,
    )
    assert ent["pose"]["x"] == 12.0
    assert ent["velocity"]["x"] == 0.0


def test_drag_slows_coast_at_hold() -> None:
    state = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0, vx=18.0, vy=0.0, vz=0.0)
    hold = {"x": 0.0, "y": 0.0, "z": 10.0, "yaw_deg": 0.0}
    lim = KinematicLimits(max_speed_mps=25.0, max_accel_mps2=30.0)
    aero = AeroEnvironment(drag_decel_per_mps=2.5)
    dt = 0.1
    speed0 = math.hypot(state.vx, state.vy, state.vz)
    for _ in range(25):
        state = integrate_toward_pose(state, hold, dt, lim, aero)
    speed1 = math.hypot(state.vx, state.vy, state.vz)
    assert speed1 < speed0 * 0.5
    assert speed1 < speed0 - 3.0


def test_wind_shifts_trajectory() -> None:
    state = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0)
    commanded = {"x": 0.0, "y": 80.0, "z": 10.0, "yaw_deg": 0.0}
    lim = KinematicLimits(max_speed_mps=20.0, max_accel_mps2=30.0)
    aero = AeroEnvironment(wind_x_mps=4.0)
    dt = 0.1
    for _ in range(40):
        state = integrate_toward_pose(state, commanded, dt, lim, aero)
    assert state.x > 1.5
    assert state.y > 5.0


def test_zero_aero_matches_default_aero() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0)
    commanded = {"x": 40.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    lim = _limits()
    dt = 0.1
    implicit = integrate_toward_pose(state, commanded, dt, lim)
    explicit = integrate_toward_pose(
        state,
        commanded,
        dt,
        lim,
        AeroEnvironment(),
    )
    assert implicit.x == pytest.approx(explicit.x)
    assert implicit.vx == pytest.approx(explicit.vx)


def test_zero_wind_drag_preserves_step2_accel_behavior() -> None:
    state = PlantState(x=0.0, y=0.0, z=0.0, yaw_deg=0.0)
    commanded = {"x": 100.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    lim = _limits()
    out = integrate_toward_pose(state, commanded, 0.1, lim, AeroEnvironment())
    speed = math.hypot(out.vx, out.vy, out.vz)
    assert speed <= lim.max_accel_mps2 * 0.1 + 1e-6
    assert speed > 0.5
