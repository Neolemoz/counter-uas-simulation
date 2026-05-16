from __future__ import annotations

import importlib.util
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_realism_failure():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'classify_realism_failure.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('classify_realism_failure', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_realism_failure_prefers_stale_induced_when_stale_and_no_recovery() -> None:
    mod = _load_realism_failure()
    row = {
        'hit': False,
        'stale_detection_count': 2,
        'delayed_detection_count': 0,
        'dropout_burst_count': 0,
        'track_coast_count': 3,
        'track_recovery_count': 0,
        'max_track_missed_frames': 2,
    }
    assert mod.classify_realism_failure(row) == 'R1_stale_induced'


def test_realism_failure_flags_coast_collapse_on_bursts_without_recovery() -> None:
    mod = _load_realism_failure()
    row = {
        'hit': False,
        'stale_detection_count': 0,
        'delayed_detection_count': 0,
        'dropout_burst_count': 1,
        'track_coast_count': 4,
        'track_recovery_count': 0,
        'max_track_missed_frames': 4,
    }
    assert mod.classify_realism_failure(row) == 'R3_coast_collapse'
