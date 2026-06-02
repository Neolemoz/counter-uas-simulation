"""Range-dependent PD and measurement noise for radar_sim / camera_sim (additive realism)."""

from __future__ import annotations

import importlib.util
import random
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_range_realism(module_path: str):  # noqa: ANN201
    path = _REPO_ROOT / module_path
    spec = importlib.util.spec_from_file_location('range_realism_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _empirical_detection_rate(p_eff: float, *, trials: int = 20_000, seed: int = 42) -> float:
    rng = random.Random(seed)
    hits = sum(1 for _ in range(trials) if rng.random() <= p_eff)
    return hits / trials


@pytest.fixture(scope='module')
def radar_rr():  # noqa: ANN201
    return _load_range_realism('src/radar_sim/radar_sim/range_realism.py')


@pytest.fixture(scope='module')
def camera_rr():  # noqa: ANN201
    return _load_range_realism('src/camera_sim/camera_sim/range_realism.py')


@pytest.mark.parametrize(
    'module_path',
    [
        'src/radar_sim/radar_sim/radar_sim_node.py',
        'src/camera_sim/camera_sim/camera_sim_node.py',
    ],
)
def test_sensor_nodes_declare_range_realism_params_default_off(module_path: str) -> None:
    source = (_REPO_ROOT / module_path).read_text(encoding='utf-8')
    assert 'detection_probability_decay_with_range' in source
    assert 'min_detection_probability' in source
    assert 'measurement_std_scale_with_range' in source
    assert 'effective_detection_probability' in source
    assert 'effective_measurement_std' in source


def test_defaults_preserve_flat_pd_and_base_sigma(radar_rr) -> None:
    base_p = 0.97
    assert (
        radar_rr.effective_detection_probability(
            base_p=base_p,
            distance_m=50.0,
            max_range_m=100.0,
            decay_with_range=0.0,
            min_detection_probability=0.1,
        )
        == base_p
    )
    assert radar_rr.effective_measurement_std(0.5, 80.0, 100.0, 0.0) == pytest.approx(0.5)


def test_near_target_higher_pd_than_far(radar_rr, camera_rr) -> None:
    kwargs = dict(
        base_p=0.9,
        max_range_m=10.0,
        decay_with_range=1.0,
        min_detection_probability=0.1,
    )
    p_near_r = radar_rr.effective_detection_probability(distance_m=0.5, **kwargs)
    p_far_r = radar_rr.effective_detection_probability(distance_m=10.0, **kwargs)
    assert p_near_r > p_far_r
    assert p_far_r == pytest.approx(0.1)

    p_near_c = camera_rr.effective_detection_probability(distance_m=0.5, **kwargs)
    p_far_c = camera_rr.effective_detection_probability(distance_m=10.0, **kwargs)
    assert p_near_c > p_far_c

    rate_near = _empirical_detection_rate(p_near_r)
    rate_far = _empirical_detection_rate(p_far_r, seed=43)
    assert rate_near > rate_far + 0.15


def test_measurement_sigma_increases_with_range(radar_rr) -> None:
    base = 0.4
    r_max = 20.0
    scale = 1.5
    std_near = radar_rr.effective_measurement_std(base, 2.0, r_max, scale)
    std_far = radar_rr.effective_measurement_std(base, 20.0, r_max, scale)
    assert std_near == pytest.approx(base * (1.0 + scale * 0.1))
    assert std_far == pytest.approx(base * (1.0 + scale * 1.0))
    assert std_far > std_near


def test_min_detection_probability_floors_far_range_pd(radar_rr) -> None:
    p_far = radar_rr.effective_detection_probability(
        base_p=0.95,
        distance_m=100.0,
        max_range_m=100.0,
        decay_with_range=2.0,
        min_detection_probability=0.25,
    )
    assert p_far == pytest.approx(0.25)
