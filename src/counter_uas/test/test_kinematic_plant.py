from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PKG_ROOT = _REPO_ROOT / "src" / "gazebo_target_sim"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from gazebo_target_sim.kinematic_plant import (  # noqa: E402
    KinematicPlantMemory,
    KinematicPlantParams,
    KinematicPlantState,
    PlantCommand,
    command_delay_samples,
    limit_velocity_delta,
    reset_plant_memory,
    step_kinematic_plant,
)


def test_command_timeout_falls_back_and_clamps_speed() -> None:
    params = KinematicPlantParams(
        dt_s=0.1,
        max_speed_m_s=5.0,
        cmd_timeout_s=0.5,
        fallback_velocity=(10.0, 0.0, 0.0),
    )
    result = step_kinematic_plant(
        KinematicPlantState(position=(0.0, 0.0, 0.0)),
        reset_plant_memory(params),
        PlantCommand(velocity=(1.0, 0.0, 0.0), has_command=True, age_s=0.6),
        params,
    )

    assert result.source == "fallback"
    assert result.selected_command == pytest.approx((5.0, 0.0, 0.0))
    assert result.state.position == pytest.approx((0.5, 0.0, 0.0))


def test_command_delay_fifo_outputs_oldest_sample() -> None:
    params = KinematicPlantParams(dt_s=0.1, max_speed_m_s=20.0, cmd_delay_s=0.2)
    assert command_delay_samples(params) == 2
    state = KinematicPlantState(position=(0.0, 0.0, 0.0))
    mem = reset_plant_memory(params)

    r1 = step_kinematic_plant(state, mem, PlantCommand((10.0, 0.0, 0.0)), params)
    r2 = step_kinematic_plant(r1.state, r1.memory, PlantCommand((10.0, 0.0, 0.0)), params)
    r3 = step_kinematic_plant(r2.state, r2.memory, PlantCommand((10.0, 0.0, 0.0)), params)

    assert r1.delayed_command == (0.0, 0.0, 0.0)
    assert r2.delayed_command == (0.0, 0.0, 0.0)
    assert r3.delayed_command == (10.0, 0.0, 0.0)
    assert r3.state.position == pytest.approx((1.0, 0.0, 0.0))


def test_first_order_autopilot_tau_matches_expected_alpha() -> None:
    params = KinematicPlantParams(dt_s=0.1, max_speed_m_s=20.0, autopilot_tau_s=0.3)
    result = step_kinematic_plant(
        KinematicPlantState(position=(0.0, 0.0, 0.0)),
        reset_plant_memory(params),
        PlantCommand((8.0, 0.0, 0.0)),
        params,
    )

    assert result.state.applied_velocity == pytest.approx((2.0, 0.0, 0.0))
    assert result.state.position == pytest.approx((0.2, 0.0, 0.0))


def test_turn_then_accel_limit_matches_live_order() -> None:
    out = limit_velocity_delta(
        previous=(10.0, 0.0, 0.0),
        desired=(0.0, 10.0, 0.0),
        max_turn_rate_rad_s=1.0,
        max_accel_m_s2=5.0,
        dt_s=0.1,
    )

    # Turn-limited desired direction is only 0.1 rad away, then the resulting
    # delta-v is capped to 0.5 m/s.
    assert math.hypot(out[0] - 10.0, out[1]) == pytest.approx(0.5, rel=1e-6)
    assert out[1] > 0.0


def test_idle_resets_memory_and_holds_position() -> None:
    params = KinematicPlantParams(dt_s=0.1, max_speed_m_s=20.0, cmd_delay_s=0.2)
    mem = KinematicPlantMemory(
        smoothed_velocity=(1.0, 2.0, 3.0),
        previous_velocity=(4.0, 5.0, 6.0),
        command_buffer=((7.0, 0.0, 0.0), (8.0, 0.0, 0.0)),
    )
    result = step_kinematic_plant(
        KinematicPlantState(position=(1.0, 2.0, 3.0), applied_velocity=(4.0, 0.0, 0.0)),
        mem,
        PlantCommand((9.0, 0.0, 0.0), idle=True),
        params,
    )

    assert result.state.position == (1.0, 2.0, 3.0)
    assert result.state.applied_velocity == (0.0, 0.0, 0.0)
    assert result.memory.smoothed_velocity == (0.0, 0.0, 0.0)
    assert result.memory.previous_velocity == (0.0, 0.0, 0.0)
    assert len(result.memory.command_buffer) == 2
