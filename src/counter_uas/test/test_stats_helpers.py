"""Layer C statistical helper regressions."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_stats():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'stats_helpers.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('stats_helpers', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_wilson_ci_is_bounded_and_centered() -> None:
    stats = _load_stats()
    ci = stats.wilson_ci(20, 40)
    assert ci['estimate'] == pytest.approx(0.5)
    assert 0.0 <= ci['lower'] < 0.5 < ci['upper'] <= 1.0
    assert ci['method'] == 'wilson'


def test_bootstrap_quantile_ci_is_deterministic() -> None:
    stats = _load_stats()
    values = [1.0, 2.0, 3.0, 4.0, 10.0]
    a = stats.bootstrap_quantile_ci(values, 95, n_resamples=200, seed=17)
    b = stats.bootstrap_quantile_ci(values, 95, n_resamples=200, seed=17)
    assert a == b
    assert a['estimate'] == pytest.approx(stats.percentile(values, 95))
    assert a['lower'] <= a['estimate'] <= a['upper']


def test_paired_binary_summary_tracks_discordant_pairs() -> None:
    stats = _load_stats()
    summary = stats.paired_binary_summary(
        [True, True, False, False],
        [True, False, True, False],
        n_resamples=100,
        seed=3,
    )
    assert summary['n_pairs'] == 4
    assert summary['baseline_only_success'] == 1
    assert summary['candidate_only_success'] == 1
    assert summary['discordant_pairs'] == 2
    assert summary['exact_mcnemar_p_two_sided'] == pytest.approx(1.0)


def test_paired_bootstrap_delta_ci_uses_candidate_minus_baseline() -> None:
    stats = _load_stats()
    ci = stats.paired_bootstrap_delta_ci(
        [2.0, 3.0, 5.0],
        [1.0, 2.0, 4.0],
        lambda xs: sum(xs) / len(xs),
        statistic_name='mean',
        n_resamples=100,
        seed=11,
    )
    assert ci['estimate'] == pytest.approx(-1.0)
    assert ci['upper'] <= 0.0
