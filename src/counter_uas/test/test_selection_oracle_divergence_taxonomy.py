from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _load_divergence_classifier():  # noqa: ANN201
    path = _REPO_ROOT / 'scripts' / 'evaluation' / 'classify_selection_oracle_divergence.py'
    assert path.is_file(), f'missing {path}'
    spec = importlib.util.spec_from_file_location('classify_selection_oracle_divergence', path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_selection_oracle_divergence_flags_clean_alignment() -> None:
    mod = _load_divergence_classifier()
    row = {
        'n_selection_blocks': 7,
        'selection_oracle_match_rate': 1.0,
        'last_oracle_match': True,
        'observer_visibility_windows': 6,
        'tracks_state_observation_count': 293,
    }
    assert mod.classify_selection_oracle_divergence(row) == 'D0_clean_aligned_selection'


def test_selection_oracle_divergence_keeps_historical_rows_inconclusive() -> None:
    mod = _load_divergence_classifier()
    assert mod.classify_selection_oracle_divergence({}) == 'D5_inconclusive_visibility_limited'
    assert mod.classify_selection_oracle_divergence(
        {
            'n_selection_blocks': 0,
            'selection_oracle_match_rate': None,
            'observer_visibility_windows': 0,
            'tracks_state_observation_count': 0,
        },
    ) == 'D5_inconclusive_visibility_limited'


def test_selection_oracle_divergence_flags_late_stage_divergence_without_churn() -> None:
    mod = _load_divergence_classifier()
    row = {
        'n_selection_blocks': 15,
        'selection_oracle_match_rate': 0.733,
        'last_oracle_match': False,
        'observer_visibility_windows': 6,
        'tracks_state_observation_count': 287,
        'candidate_discard_count': 2,
        'track_continuity_change_count': 3,
        'recovery_thrash_count': 4,
        'selection_proxy_event_count': 1,
        'ambiguity_failure_class': 'A0_none',
        'realism_failure_class': 'R0_none',
    }
    assert mod.classify_selection_oracle_divergence(row) == 'D1_late_stage_divergence'


def test_selection_oracle_divergence_flags_persistent_mismatch() -> None:
    mod = _load_divergence_classifier()
    row = {
        'n_selection_blocks': 8,
        'selection_oracle_match_rate': 0.5,
        'last_oracle_match': False,
        'observer_visibility_windows': 4,
        'tracks_state_observation_count': 100,
        'candidate_discard_count': 0,
        'track_continuity_change_count': 0,
        'recovery_thrash_count': 0,
        'selection_proxy_event_count': 0,
        'ambiguity_failure_class': 'A0_none',
        'realism_failure_class': 'R0_none',
    }
    assert mod.classify_selection_oracle_divergence(row) == 'D2_persistent_oracle_mismatch'


def test_selection_oracle_divergence_flags_wave6_fragmented_failure_as_churn_driven() -> None:
    mod = _load_divergence_classifier()
    row = {
        'n_selection_blocks': 15,
        'selection_oracle_match_rate': 0.7333333333333333,
        'last_oracle_match': False,
        'observer_visibility_windows': 6,
        'tracks_state_observation_count': 287,
        'candidate_discard_count': 17,
        'track_continuity_change_count': 19,
        'recovery_thrash_count': 32,
        'selection_proxy_event_count': 10,
        'ambiguity_failure_class': 'A3_merge_instability',
        'realism_failure_class': 'R4_timing_divergence',
    }
    assert mod.classify_selection_oracle_divergence(row) == 'D3_churn_driven_instability'


def test_selection_oracle_divergence_flags_deletion_dominated_collapse() -> None:
    mod = _load_divergence_classifier()
    row = {
        'n_selection_blocks': 5,
        'selection_oracle_match_rate': 0.8,
        'last_oracle_match': False,
        'observer_visibility_windows': 3,
        'tracks_state_observation_count': 80,
        'track_deleted_count': 25,
        'candidate_discard_count': 2,
        'track_continuity_change_count': 2,
        'recovery_thrash_count': 2,
        'selection_proxy_event_count': 1,
        'ambiguity_failure_class': 'A0_none',
        'realism_failure_class': 'R0_none',
    }
    assert mod.classify_selection_oracle_divergence(row) == 'D4_deletion_dominated_collapse'
