"""RT sandbox vs offline simulation engagement limit parity (Step 4)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_SIM = _REPO / 'simulation'
_GZ_PKG = _REPO / 'src' / 'rt_sandbox_gz'
_BRIDGE_PKG = _REPO / 'platform' / 'rt-sandbox-bridge'
for path in (_SIM, _GZ_PKG, _BRIDGE_PKG):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from engagement_limits import default_engagement_limits, load_engagement_limits  # noqa: E402
from rt_plant_adapter import (  # noqa: E402
    aero_from_engagement,
    integrate_commanded_pose,
    limits_from_engagement,
)
from rt_sandbox.engagement_limits import default_engagement_limits as bridge_defaults  # noqa: E402
from rt_sandbox.kinematic_plant import PlantState  # noqa: E402
from rt_sandbox_gz.engagement_limits import (  # noqa: E402
    default_aero_environment,
    default_kinematic_limits,
)
import realtime_sim  # noqa: E402


def test_yaml_tuple_matches_rt_and_bridge_defaults() -> None:
    el = load_engagement_limits()
    rt_lim = default_kinematic_limits()
    rt_aero = default_aero_environment()
    br = bridge_defaults()

    assert rt_lim.max_speed_mps == el.max_speed_mps == br.max_speed_mps
    assert rt_lim.max_accel_mps2 == el.max_accel_mps2 == br.max_accel_mps2
    assert rt_lim.max_turn_rate_rad_s == pytest.approx(el.max_turn_rate_rad_s)
    assert rt_lim.max_climb_mps == el.max_climb_mps
    assert rt_aero.drag_decel_per_mps == el.drag_decel_per_mps
    assert rt_aero.wind_x_mps == el.wind_x_mps == 0.0


def test_offline_realtime_sim_turn_rate_matches_tuple() -> None:
    el = default_engagement_limits()
    assert realtime_sim.MAX_TURN_RATE_DEG_S == pytest.approx(el.turn_rate_deg_s)
    assert math.radians(realtime_sim.MAX_TURN_RATE_DEG_S) == pytest.approx(
        el.max_turn_rate_rad_s
    )


def test_rt_offline_shared_integrator_trajectory_tolerance() -> None:
    el = default_engagement_limits()
    state = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0)
    cmd = {"x": 60.0, "y": 20.0, "z": 12.0, "yaw_deg": 15.0}
    dt = 0.1
    lim = limits_from_engagement(el)
    aero = aero_from_engagement(el)

    from rt_sandbox_gz.kinematic_plant import integrate_toward_pose

    s_rt = state
    s_off = state
    for _ in range(30):
        s_rt = integrate_toward_pose(s_rt, cmd, dt, lim, aero)
        s_off = integrate_commanded_pose(s_off, cmd, dt, el)
    assert s_rt.x == pytest.approx(s_off.x, abs=1e-9)
    assert s_rt.y == pytest.approx(s_off.y, abs=1e-9)
    assert s_rt.z == pytest.approx(s_off.z, abs=1e-9)
    assert s_rt.vx == pytest.approx(s_off.vx, abs=1e-9)


def test_drag_parity_coast_slowdown() -> None:
    el = load_engagement_limits()
    assert el.drag_decel_per_mps > 0.0
    state = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0, vx=16.0, vy=0.0, vz=0.0)
    speed0 = math.hypot(state.vx, state.vy, state.vz)
    for _ in range(25):
        hold = state.as_pose()
        state = integrate_commanded_pose(state, hold, 0.1, el)
    speed1 = math.hypot(state.vx, state.vy, state.vz)
    assert speed1 < speed0 * 0.75


def test_wind_parity_lateral_drift() -> None:
    el = load_engagement_limits()
    windy = type(el)(
        max_speed_mps=el.max_speed_mps,
        max_accel_mps2=el.max_accel_mps2,
        max_turn_rate_rad_s=el.max_turn_rate_rad_s,
        max_climb_mps=el.max_climb_mps,
        drag_decel_per_mps=el.drag_decel_per_mps,
        wind_x_mps=3.0,
        wind_y_mps=0.0,
        wind_z_mps=0.0,
        max_turn_rate_deg_s=el.max_turn_rate_deg_s,
    )
    calm = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0)
    wind = PlantState(x=0.0, y=0.0, z=10.0, yaw_deg=0.0)
    for _ in range(20):
        calm = integrate_commanded_pose(calm, calm.as_pose(), 0.1, el)
        wind = integrate_commanded_pose(wind, wind.as_pose(), 0.1, windy)
    assert wind.x > calm.x + 0.4
    assert el.wind_x_mps == 0.0
