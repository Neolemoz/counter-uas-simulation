from __future__ import annotations

import importlib.util
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_ambiguity_failure():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'classify_ambiguity_failure.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('classify_ambiguity_failure', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_ambiguity_failure_prefers_ghost_induced_churn() -> None:
    mod = _load_ambiguity_failure()
    row = {
        'hit': False,
        'ghost_detection_count': 3,
        'fragmented_gap_count': 0,
        'candidate_spawn_count': 5,
        'candidate_discard_count': 4,
        'duplicate_track_merge_count': 0,
        'track_coast_count': 0,
        'track_recovery_count': 0,
        'max_track_missed_frames': 0,
        'n_selection_blocks': 0,
        'selection_oracle_match_rate': None,
        'notes_meta': '',
    }
    assert mod.classify_ambiguity_failure(row) == 'A1_ghost_induced_churn'


def test_ambiguity_failure_flags_crossing_proxy_instability() -> None:
    mod = _load_ambiguity_failure()
    row = {
        'hit': False,
        'ghost_detection_count': 1,
        'fragmented_gap_count': 1,
        'candidate_spawn_count': 3,
        'candidate_discard_count': 1,
        'duplicate_track_merge_count': 0,
        'track_coast_count': 1,
        'track_recovery_count': 1,
        'max_track_missed_frames': 1,
        'n_selection_blocks': 4,
        'selection_oracle_match_rate': 0.75,
        'notes_meta': 'scenario=multi crossing proxy ambiguity profile',
    }
    assert mod.classify_ambiguity_failure(row) == 'A4_crossing_association_instability'
