"""Golden cases for the canonical constant-velocity intercept solver.

These tests pin the numerical contract shared by the live Gazebo node and offline
simulation tools.  The canonical implementation lives in ``guidance_lib.py`` and
uses a tolerance-scaled geometric closure check rather than a strict machine-epsilon
comparison.
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_guidance_lib():  # noqa: ANN201
    path = _REPO_ROOT / 'src' / 'gazebo_target_sim' / 'gazebo_target_sim' / 'guidance_lib.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('guidance_lib', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _load_sim_intercept():  # noqa: ANN201
    path = _REPO_ROOT / 'simulation' / 'core' / 'intercept.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('sim_intercept', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


@dataclass(frozen=True)
class SolverCase:
    name: str
    p_t: tuple[float, float, float]
    v_t: tuple[float, float, float]
    p_i: tuple[float, float, float]
    speed: float
    expected_t: float | None


GOLDEN_CASES = [
    SolverCase(
        name='head_on_closing',
        p_t=(100.0, 0.0, 0.0),
        v_t=(-10.0, 0.0, 0.0),
        p_i=(0.0, 0.0, 0.0),
        speed=20.0,
        expected_t=100.0 / 30.0,
    ),
    SolverCase(
        name='linear_branch_equal_speeds_approaching',
        p_t=(10.0, 0.0, 0.0),
        v_t=(-5.0, 0.0, 0.0),
        p_i=(0.0, 0.0, 0.0),
        speed=5.0,
        expected_t=1.0,
    ),
    SolverCase(
        name='crossing_intercept',
        p_t=(0.0, 100.0, 0.0),
        v_t=(10.0, 0.0, 0.0),
        p_i=(0.0, 0.0, 0.0),
        speed=20.0,
        expected_t=(10000.0 / 300.0) ** 0.5,
    ),
    SolverCase(
        name='too_fast_moving_away',
        p_t=(0.0, 0.0, 0.0),
        v_t=(30.0, 0.0, 0.0),
        p_i=(-100.0, 0.0, 0.0),
        speed=10.0,
        expected_t=None,
    ),
    SolverCase(
        name='crossing_discriminant_negative',
        p_t=(0.0, 100.0, 0.0),
        v_t=(30.0, 0.0, 0.0),
        p_i=(0.0, 0.0, 0.0),
        speed=20.0,
        expected_t=None,
    ),
    SolverCase(
        name='coincident_start_has_no_positive_root',
        p_t=(0.0, 0.0, 0.0),
        v_t=(1.0, 0.0, 0.0),
        p_i=(0.0, 0.0, 0.0),
        speed=5.0,
        expected_t=None,
    ),
]


@pytest.mark.parametrize('case', GOLDEN_CASES, ids=[c.name for c in GOLDEN_CASES])
def test_solve_intercept_time_golden_cases(case: SolverCase) -> None:
    lib = _load_guidance_lib()
    t = lib.solve_intercept_time(
        *case.p_t,
        *case.v_t,
        *case.p_i,
        case.speed,
    )
    if case.expected_t is None:
        assert t is None
    else:
        assert t is not None
        assert t == pytest.approx(case.expected_t, rel=1e-9, abs=1e-9)


@pytest.mark.parametrize(
    'case',
    [c for c in GOLDEN_CASES if c.expected_t is not None],
    ids=[c.name for c in GOLDEN_CASES if c.expected_t is not None],
)
def test_compute_intercept_golden_cases_close_geometry(case: SolverCase) -> None:
    lib = _load_guidance_lib()
    sol = lib.compute_intercept(
        *case.p_t,
        *case.v_t,
        *case.p_i,
        case.speed,
    )
    assert sol is not None
    t, phx, phy, phz, ux, uy, uz = sol
    assert t == pytest.approx(case.expected_t, rel=1e-9, abs=1e-9)
    assert (phx, phy, phz) == pytest.approx(
        (
            case.p_t[0] + case.v_t[0] * t,
            case.p_t[1] + case.v_t[1] * t,
            case.p_t[2] + case.v_t[2] * t,
        ),
        rel=1e-9,
        abs=1e-9,
    )
    assert lib.norm(ux, uy, uz) == pytest.approx(1.0, rel=1e-9, abs=1e-9)


@pytest.mark.parametrize('case', GOLDEN_CASES, ids=[c.name for c in GOLDEN_CASES])
def test_simulation_core_delegates_to_guidance_contract(case: SolverCase) -> None:
    lib = _load_guidance_lib()
    sim = _load_sim_intercept()
    expected = lib.solve_intercept_time(
        *case.p_t,
        *case.v_t,
        *case.p_i,
        case.speed,
    )
    actual = sim.solve_intercept_time(
        np.array(case.p_t, dtype=float),
        np.array(case.v_t, dtype=float),
        np.array(case.p_i, dtype=float),
        case.speed,
    )
    assert actual == expected
