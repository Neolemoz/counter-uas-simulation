"""Sensor timing realism: decimation and transport delay helpers."""

from __future__ import annotations

import importlib.util
import random
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_timing(module_path: str):  # noqa: ANN201
    path = _REPO_ROOT / module_path
    spec = importlib.util.spec_from_file_location('timing_realism_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def timing():  # noqa: ANN201
    return _load_timing('src/radar_sim/radar_sim/timing_realism.py')


def test_publish_every_n_default_publishes_every_callback(timing) -> None:
    for idx in range(1, 8):
        assert timing.should_publish_on_callback(idx, 1) is True


def test_publish_every_two_skips_alternate_callbacks(timing) -> None:
    flags = [timing.should_publish_on_callback(i, 2) for i in range(1, 7)]
    assert flags == [False, True, False, True, False, True]


def test_transport_delay_without_jitter_is_exact_mean(timing) -> None:
    assert timing.transport_delay_s(0.12, 0.0, None) == pytest.approx(0.12)


def test_transport_delay_with_seeded_jitter_is_repeatable(timing) -> None:
    rng_a = random.Random(77)
    rng_b = random.Random(77)
    samples_a = [timing.transport_delay_s(0.10, 0.04, rng_a) for _ in range(5)]
    samples_b = [timing.transport_delay_s(0.10, 0.04, rng_b) for _ in range(5)]
    assert samples_a == samples_b
    assert all(0.06 <= s <= 0.14 for s in samples_a)


def test_transport_delay_positive_mean_schedules_timer_path_in_nodes() -> None:
    for module_path in (
        'src/radar_sim/radar_sim/radar_sim_node.py',
        'src/camera_sim/camera_sim/camera_sim_node.py',
    ):
        source = (_REPO_ROOT / module_path).read_text(encoding='utf-8')
        assert "declare_parameter('radar.publish_every_n', 1)" in source or (
            "declare_parameter('camera.publish_every_n', 1)" in source
        )
        assert 'transport_delay_s' in source
        assert 'create_timer(delay_s, _fire)' in source


def test_default_config_yaml_omits_timing_overrides() -> None:
    for name in ('config.yaml', 'config_gazebo_counter_uas.yaml', 'config_lab_toy.yaml'):
        text = (_REPO_ROOT / 'src' / 'counter_uas' / 'config' / name).read_text(encoding='utf-8')
        assert 'publish_every_n' not in text
        assert 'delay_mean_s' not in text
