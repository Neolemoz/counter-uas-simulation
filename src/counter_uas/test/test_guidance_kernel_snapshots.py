"""Layer A guidance-kernel characterization tests.

These snapshot deterministic ``GuidanceInput -> GuidanceCommand`` behavior
before live-node extraction.  They intentionally avoid ROS nodes, wall-clock
time, random state, marker assertions, and log assertions.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PKG_ROOT = _REPO_ROOT / "src" / "gazebo_target_sim"
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from gazebo_target_sim.guidance_kernel import (  # noqa: E402
    GuidanceInput,
    GuidanceMemory,
    GuidanceParams,
    InterceptorKinematics,
    TargetKinematics,
    compute_guidance_command,
    pn_inspired_steering_vector,
)

Vec3 = tuple[float, float, float]

REL_TOL = 1e-9
ABS_TOL = 1e-9


@dataclass(frozen=True)
class MemorySnapshot:
    mode: str
    predict_valid_count: int
    predict_invalid_count: int
    filtered_t_go_s: float | None
    smoothed_intercept_point: Vec3 | None
    previous_unit_command: Vec3 | None


@dataclass(frozen=True)
class CommandSnapshot:
    velocity_cmd: Vec3
    unit_cmd: Vec3
    speed_cmd_m_s: float
    mode: str
    solution_valid: bool
    t_go_raw_s: float | None
    t_go_effective_s: float | None
    intercept_point_raw: Vec3 | None
    intercept_point_smoothed: Vec3 | None
    memory_next: MemorySnapshot
    pn_inspired_active: bool = False
    closing_speed_m_s: float | None = None
    mode_transition: tuple[str, str] | None = ("pursuit", "predict")


@dataclass(frozen=True)
class SnapshotCase:
    name: str
    guidance_input: GuidanceInput
    expected: CommandSnapshot


def _params(**overrides):  # noqa: ANN003, ANN202
    base = dict(
        interceptor_max_speed_m_s=80.0,
        max_speed_m_s=60.0,
        speed_k1=0.15,
        speed_k2=0.5,
        speed_vmin_m_s=1.0,
        speed_tgo_min_s=0.2,
        t_hit_min_s=0.02,
        t_hit_max_s=120.0,
        pursuit_lead_blend=0.0,
        naive_lead_time_s=0.85,
        predict_enter_frames=1,
        predict_exit_frames=1,
        intercept_smoothing_alpha=1.0,
        align_speed_to_solver=True,
        t_go_filter_alpha=0.5,
        t_go_filter_max_step_s=0.0,
        align_speed_use_smooth_hit_range=False,
        guidance_u_max_step_rad=math.pi,
        guidance_terminal_range_m=0.0,
        guidance_terminal_pursuit_blend=0.0,
        use_pn_inspired=False,
        pn_navigation_constant=3.0,
        pn_blend_gain=0.0,
        pn_blend_terminal_range_m=0.0,
        pn_min_closing_speed_m_s=0.15,
    )
    base.update(overrides)
    return GuidanceParams(**base)


def _input(
    *,
    target: Vec3 = (100.0, 0.0, 0.0),
    target_vel: Vec3 = (-10.0, 0.0, 0.0),
    interceptor: Vec3 = (0.0, 0.0, 0.0),
    interceptor_vel: Vec3 = (0.0, 0.0, 0.0),
    params: GuidanceParams | None = None,
    memory: GuidanceMemory | None = None,
    aim: Vec3 | None = None,
    measurement_delay_s: float = 0.0,
) -> GuidanceInput:
    return GuidanceInput(
        target=TargetKinematics(target, target_vel, measurement_delay_s),
        interceptor=InterceptorKinematics(interceptor, interceptor_vel),
        params=params or _params(),
        memory=memory or GuidanceMemory(),
        aim_point_override=aim,
    )


def _point_style_ema_velocity(
    previous: Vec3,
    current: Vec3,
    *,
    dt_s: float,
    alpha: float,
    previous_smooth: Vec3 = (0.0, 0.0, 0.0),
) -> Vec3:
    raw = (
        (current[0] - previous[0]) / dt_s,
        (current[1] - previous[1]) / dt_s,
        (current[2] - previous[2]) / dt_s,
    )
    return (
        alpha * raw[0] + (1.0 - alpha) * previous_smooth[0],
        alpha * raw[1] + (1.0 - alpha) * previous_smooth[1],
        alpha * raw[2] + (1.0 - alpha) * previous_smooth[2],
    )


def _assert_float(actual: float | None, expected: float | None) -> None:
    if expected is None:
        assert actual is None
    else:
        assert actual == pytest.approx(expected, rel=REL_TOL, abs=ABS_TOL)


def _assert_vec(actual: Vec3 | None, expected: Vec3 | None) -> None:
    if expected is None:
        assert actual is None
    else:
        assert actual == pytest.approx(expected, rel=REL_TOL, abs=ABS_TOL)


def _assert_memory(actual: GuidanceMemory, expected: MemorySnapshot) -> None:
    assert actual.mode == expected.mode
    assert actual.predict_valid_count == expected.predict_valid_count
    assert actual.predict_invalid_count == expected.predict_invalid_count
    _assert_float(actual.filtered_t_go_s, expected.filtered_t_go_s)
    _assert_vec(actual.smoothed_intercept_point, expected.smoothed_intercept_point)
    _assert_vec(actual.previous_unit_command, expected.previous_unit_command)


def _assert_command_snapshot(actual, expected: CommandSnapshot) -> None:  # noqa: ANN001
    _assert_vec(actual.velocity_cmd, expected.velocity_cmd)
    _assert_vec(actual.unit_cmd, expected.unit_cmd)
    _assert_float(actual.speed_cmd_m_s, expected.speed_cmd_m_s)
    assert actual.mode == expected.mode
    assert actual.solution_valid is expected.solution_valid
    _assert_float(actual.t_go_raw_s, expected.t_go_raw_s)
    _assert_float(actual.t_go_effective_s, expected.t_go_effective_s)
    _assert_vec(actual.intercept_point_raw, expected.intercept_point_raw)
    _assert_vec(actual.intercept_point_smoothed, expected.intercept_point_smoothed)
    _assert_memory(actual.memory_next, expected.memory_next)
    assert actual.pn_inspired_active is expected.pn_inspired_active
    _assert_float(actual.closing_speed_m_s, expected.closing_speed_m_s)
    assert actual.mode_transition == expected.mode_transition


POINT_STYLE_TARGET_VEL = _point_style_ema_velocity(
    previous=(101.0, 0.0, 0.0),
    current=(100.0, 0.0, 0.0),
    dt_s=0.1,
    alpha=0.52,
)


SNAPSHOT_CASES = [
    SnapshotCase(
        name="head_on_predictive_intercept",
        guidance_input=_input(),
        expected=CommandSnapshot(
            velocity_cmd=(27.500000000000004, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=27.500000000000004,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.6666666666666665,
            t_go_effective_s=2.6666666666666665,
            intercept_point_raw=(73.33333333333334, 0.0, 0.0),
            intercept_point_smoothed=(73.33333333333334, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=2.6666666666666665,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
        ),
    ),
    SnapshotCase(
        name="no_solution_pursuit_fallback",
        guidance_input=_input(
            target=(100.0, 0.0, 0.0),
            target_vel=(100.0, 0.0, 0.0),
            memory=GuidanceMemory(mode="predict", filtered_t_go_s=7.0),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(15.0, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=15.0,
            mode="pursuit",
            solution_valid=False,
            t_go_raw_s=None,
            t_go_effective_s=None,
            intercept_point_raw=None,
            intercept_point_smoothed=None,
            memory_next=MemorySnapshot(
                mode="pursuit",
                predict_valid_count=0,
                predict_invalid_count=1,
                filtered_t_go_s=None,
                smoothed_intercept_point=None,
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=("predict", "pursuit"),
        ),
    ),
    SnapshotCase(
        name="delayed_target_measurement",
        guidance_input=_input(measurement_delay_s=0.5),
        expected=CommandSnapshot(
            velocity_cmd=(26.375000000000004, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=26.375000000000004,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.611683848797251,
            t_go_effective_s=2.611683848797251,
            intercept_point_raw=(68.8831615120275, 0.0, 0.0),
            intercept_point_smoothed=(68.8831615120275, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=2.611683848797251,
                smoothed_intercept_point=(68.8831615120275, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
        ),
    ),
    SnapshotCase(
        name="point_style_finite_difference_velocity",
        guidance_input=_input(target_vel=POINT_STYLE_TARGET_VEL),
        expected=CommandSnapshot(
            velocity_cmd=(25.1, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=25.1,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=3.3003300330033003,
            t_go_effective_s=3.3003300330033003,
            intercept_point_raw=(82.83828382838284, 0.0, 0.0),
            intercept_point_smoothed=(82.83828382838284, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=3.3003300330033003,
                smoothed_intercept_point=(82.83828382838284, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
        ),
    ),
    SnapshotCase(
        name="tracks_state_direct_velocity",
        guidance_input=_input(target_vel=(-10.0, 1.5, -0.25)),
        expected=CommandSnapshot(
            velocity_cmd=(27.41922184560437, 1.5000000000000002, -0.25000000000000006),
            unit_cmd=(0.9984655914086829, 0.05462220611315866, -0.009103701018859779),
            speed_cmd_m_s=27.461358790461695,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.672423291232792,
            t_go_effective_s=2.672423291232792,
            intercept_point_raw=(73.27576708767208, 4.008634936849187, -0.668105822808198),
            intercept_point_smoothed=(73.27576708767208, 4.008634936849187, -0.668105822808198),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=2.672423291232792,
                smoothed_intercept_point=(73.27576708767208, 4.008634936849187, -0.668105822808198),
                previous_unit_command=(0.9984655914086829, 0.05462220611315866, -0.009103701018859779),
            ),
        ),
    ),
    SnapshotCase(
        name="terminal_blend_transition",
        guidance_input=_input(
            target=(100.0, 100.0, 0.0),
            target_vel=(-5.0, 0.0, 0.0),
            params=_params(guidance_terminal_range_m=200.0, guidance_terminal_pursuit_blend=1.0),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(21.808948905285437, 25.348594241564818, 0.0),
            unit_cmd=(0.6521966850766292, 0.7580497899050276, 0.0),
            speed_cmd_m_s=33.43922072023874,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=3.844295927207804,
            t_go_effective_s=3.844295927207804,
            intercept_point_raw=(80.77852036396098, 100.0, 0.0),
            intercept_point_smoothed=(80.77852036396098, 100.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=3.844295927207804,
                smoothed_intercept_point=(80.77852036396098, 100.0, 0.0),
                previous_unit_command=(0.6521966850766292, 0.7580497899050276, 0.0),
            ),
        ),
    ),
    SnapshotCase(
        name="pn_inspired_steering_blend",
        guidance_input=_input(
            target=(100.0, 0.0, 0.0),
            target_vel=(-10.0, 5.0, 0.0),
            params=_params(use_pn_inspired=True, pn_blend_gain=0.12, align_speed_to_solver=False),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(31.222267536128808, 11.586067507648039, 0.0),
            unit_cmd=(0.9375308317678567, 0.34790219816015966, 0.0),
            speed_cmd_m_s=33.30265680676814,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.731843826165745,
            t_go_effective_s=None,
            intercept_point_raw=(72.68156173834255, 13.659219130828726, 0.0),
            intercept_point_smoothed=(72.68156173834255, 13.659219130828726, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=None,
                smoothed_intercept_point=(72.68156173834255, 13.659219130828726, 0.0),
                previous_unit_command=(0.9375308317678567, 0.34790219816015966, 0.0),
            ),
            pn_inspired_active=True,
            closing_speed_m_s=10.0,
        ),
    ),
    SnapshotCase(
        name="unit_command_slew_limiting",
        guidance_input=_input(
            target=(0.0, 100.0, 0.0),
            target_vel=(0.0, 0.0, 0.0),
            params=_params(guidance_u_max_step_rad=0.25, align_speed_to_solver=False),
            memory=GuidanceMemory(previous_unit_command=(1.0, 0.0, 0.0)),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(25.433951069904428, 6.494353930431227, 0.0),
            unit_cmd=(0.9689124217106447, 0.2474039592545229, 0.0),
            speed_cmd_m_s=26.250000000000004,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=4.444444444444445,
            t_go_effective_s=None,
            intercept_point_raw=(0.0, 100.0, 0.0),
            intercept_point_smoothed=(0.0, 100.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=None,
                smoothed_intercept_point=(0.0, 100.0, 0.0),
                previous_unit_command=(0.9689124217106447, 0.2474039592545229, 0.0),
            ),
        ),
    ),
    SnapshotCase(
        name="t_go_filtering_behavior",
        guidance_input=_input(
            params=_params(t_go_filter_alpha=1.0, t_go_filter_max_step_s=0.5),
            memory=GuidanceMemory(mode="predict", filtered_t_go_s=10.0),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(7.719298245614036, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=7.719298245614036,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.6666666666666665,
            t_go_effective_s=9.5,
            intercept_point_raw=(73.33333333333334, 0.0, 0.0),
            intercept_point_smoothed=(73.33333333333334, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=9.5,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=None,
        ),
    ),
    SnapshotCase(
        name="intercept_point_smoothing_behavior",
        guidance_input=_input(
            target=(100.0, 100.0, 0.0),
            target_vel=(-5.0, 0.0, 0.0),
            params=_params(intercept_smoothing_alpha=0.25, align_speed_use_smooth_hit_range=True),
            memory=GuidanceMemory(smoothed_intercept_point=(80.0, 80.0, 0.0)),
        ),
        expected=CommandSnapshot(
            velocity_cmd=(20.860680761701232, 22.110680761701232, 0.0),
            unit_cmd=(0.6862473698689913, 0.7273682336670279, 0.0),
            speed_cmd_m_s=30.39819411720829,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=3.844295927207804,
            t_go_effective_s=3.844295927207804,
            intercept_point_raw=(80.77852036396098, 100.0, 0.0),
            intercept_point_smoothed=(80.19463009099024, 85.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=3.844295927207804,
                smoothed_intercept_point=(80.19463009099024, 85.0, 0.0),
                previous_unit_command=(0.6862473698689913, 0.7273682336670279, 0.0),
            ),
        ),
    ),
]


@pytest.mark.parametrize("case", SNAPSHOT_CASES, ids=[case.name for case in SNAPSHOT_CASES])
def test_guidance_command_snapshots(case: SnapshotCase) -> None:
    cmd = compute_guidance_command(case.guidance_input)

    _assert_command_snapshot(cmd, case.expected)


def test_pn_inspired_steering_semantics_are_direction_blend_inputs() -> None:
    steer, vc, ok = pn_inspired_steering_vector(
        r=(100.0, 0.0, 0.0),
        v_rel=(-10.0, 5.0, 0.0),
        navigation_constant=3.0,
        min_closing_speed_m_s=0.15,
    )

    assert ok is True
    assert vc == pytest.approx(10.0, rel=REL_TOL, abs=ABS_TOL)
    assert steer == pytest.approx((0.0, 1.5, 0.0), rel=REL_TOL, abs=ABS_TOL)


def test_memory_transition_across_sequential_ticks() -> None:
    params = _params(predict_enter_frames=2, predict_exit_frames=2)

    tick1 = compute_guidance_command(_input(params=params))
    _assert_command_snapshot(
        tick1,
        CommandSnapshot(
            velocity_cmd=(27.500000000000004, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=27.500000000000004,
            mode="pursuit",
            solution_valid=True,
            t_go_raw_s=2.6666666666666665,
            t_go_effective_s=2.6666666666666665,
            intercept_point_raw=(73.33333333333334, 0.0, 0.0),
            intercept_point_smoothed=(73.33333333333334, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="pursuit",
                predict_valid_count=1,
                predict_invalid_count=0,
                filtered_t_go_s=2.6666666666666665,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=None,
        ),
    )

    tick2 = compute_guidance_command(_input(params=params, memory=tick1.memory_next))
    _assert_command_snapshot(
        tick2,
        CommandSnapshot(
            velocity_cmd=(27.500000000000004, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=27.500000000000004,
            mode="predict",
            solution_valid=True,
            t_go_raw_s=2.6666666666666665,
            t_go_effective_s=2.6666666666666665,
            intercept_point_raw=(73.33333333333334, 0.0, 0.0),
            intercept_point_smoothed=(73.33333333333334, 0.0, 0.0),
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=2,
                predict_invalid_count=0,
                filtered_t_go_s=2.6666666666666665,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=("pursuit", "predict"),
        ),
    )

    no_solution_input = dict(
        target=(100.0, 0.0, 0.0),
        target_vel=(100.0, 0.0, 0.0),
        params=params,
    )
    tick3 = compute_guidance_command(_input(**no_solution_input, memory=tick2.memory_next))
    _assert_command_snapshot(
        tick3,
        CommandSnapshot(
            velocity_cmd=(15.0, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=15.0,
            mode="predict",
            solution_valid=False,
            t_go_raw_s=None,
            t_go_effective_s=None,
            intercept_point_raw=None,
            intercept_point_smoothed=None,
            memory_next=MemorySnapshot(
                mode="predict",
                predict_valid_count=0,
                predict_invalid_count=1,
                filtered_t_go_s=None,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=None,
        ),
    )

    tick4 = compute_guidance_command(_input(**no_solution_input, memory=tick3.memory_next))
    _assert_command_snapshot(
        tick4,
        CommandSnapshot(
            velocity_cmd=(15.0, 0.0, 0.0),
            unit_cmd=(1.0, 0.0, 0.0),
            speed_cmd_m_s=15.0,
            mode="pursuit",
            solution_valid=False,
            t_go_raw_s=None,
            t_go_effective_s=None,
            intercept_point_raw=None,
            intercept_point_smoothed=None,
            memory_next=MemorySnapshot(
                mode="pursuit",
                predict_valid_count=0,
                predict_invalid_count=2,
                filtered_t_go_s=None,
                smoothed_intercept_point=(73.33333333333334, 0.0, 0.0),
                previous_unit_command=(1.0, 0.0, 0.0),
            ),
            mode_transition=("predict", "pursuit"),
        ),
    )
