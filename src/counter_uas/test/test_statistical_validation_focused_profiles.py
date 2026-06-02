"""Contract tests for focused N=40 statistical validation profiles."""

from __future__ import annotations

import csv
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_PROFILES = (
    _REPO_ROOT
    / 'scripts'
    / 'evaluation'
    / 'fixtures'
    / 'statistical_validation_focused_n40_profiles.csv'
)


def test_focused_n40_profiles_are_four_matched_seed_pairs() -> None:
    rows = list(csv.DictReader(_PROFILES.open(encoding='utf-8', newline='')))
    labels = [str(row['label']).strip() for row in rows]
    assert labels == [
        'predictive_baseline',
        'predictive_intercept',
        'hysteresis_off',
        'hysteresis_on',
    ]
    assert all(str(row['n']).strip() == '40' for row in rows)
    assert all(str(row['seed_base']).strip() == '6201' for row in rows)
    assert all(str(row['scenario']).strip() == 'single' for row in rows)
