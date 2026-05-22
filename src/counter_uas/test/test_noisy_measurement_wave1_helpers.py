from __future__ import annotations

import importlib.util
import random
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_noisy_measurement():  # noqa: ANN201
    path = _REPO_ROOT / 'src' / 'gazebo_target_sim' / 'gazebo_target_sim' / 'noisy_measurement_node.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('noisy_measurement_node', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_near_threshold_ghost_sampling_is_seed_deterministic() -> None:
    mod = _load_noisy_measurement()
    rng_a = random.Random(77)
    rng_b = random.Random(77)
    vals_a = [
        mod._sample_ghost_radius_m(
            rng_a,
            placement_mode='near_threshold',
            broad_min_m=3.0,
            broad_max_m=12.0,
            near_threshold_min_m=18.0,
            near_threshold_max_m=24.0,
        )
        for _ in range(3)
    ]
    vals_b = [
        mod._sample_ghost_radius_m(
            rng_b,
            placement_mode='near_threshold',
            broad_min_m=3.0,
            broad_max_m=12.0,
            near_threshold_min_m=18.0,
            near_threshold_max_m=24.0,
        )
        for _ in range(3)
    ]
    assert vals_a == vals_b
    assert all(18.0 <= v <= 24.0 for v in vals_a)


def test_near_threshold_ghost_offset_sampling_is_seed_deterministic() -> None:
    mod = _load_noisy_measurement()
    rng_a = random.Random(91)
    rng_b = random.Random(91)
    vals_a = [
        mod._sample_ghost_offset(
            rng_a,
            placement_mode='near_threshold',
            broad_min_m=3.0,
            broad_max_m=12.0,
            near_threshold_min_m=18.0,
            near_threshold_max_m=24.0,
            z_std_m=0.5,
        )
        for _ in range(2)
    ]
    vals_b = [
        mod._sample_ghost_offset(
            rng_b,
            placement_mode='near_threshold',
            broad_min_m=3.0,
            broad_max_m=12.0,
            near_threshold_min_m=18.0,
            near_threshold_max_m=24.0,
            z_std_m=0.5,
        )
        for _ in range(2)
    ]
    assert vals_a == vals_b
    assert all(18.0 <= v.radius_m <= 24.0 for v in vals_a)


def test_staggered_fragmentation_schedule_is_deterministic_and_default_safe() -> None:
    mod = _load_noisy_measurement()
    starts = [
        tick
        for tick in range(1, 16)
        if mod._staggered_fragmentation_should_start(
            tick_index=tick,
            cycle_ticks=6,
            phase_ticks=1,
        )
    ]
    assert starts == [1, 7, 13]
    assert not mod._staggered_fragmentation_should_start(
        tick_index=0,
        cycle_ticks=6,
        phase_ticks=1,
    )


def test_staggered_fragmentation_schedule_supports_longer_wave2_gap_regime() -> None:
    mod = _load_noisy_measurement()
    starts = [
        tick
        for tick in range(1, 21)
        if mod._staggered_fragmentation_should_start(
            tick_index=tick,
            cycle_ticks=5,
            phase_ticks=1,
        )
    ]
    assert starts == [1, 6, 11, 16]
