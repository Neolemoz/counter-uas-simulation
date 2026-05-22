"""Offline failure bucket rules (evaluation harness)."""

from __future__ import annotations

import tempfile
from pathlib import Path
import sys

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / 'scripts' / 'evaluation'
sys.path.insert(0, str(_EVAL))

from classify_run import classify_run_failure  # noqa: E402


def test_f1_timeout_marker() -> None:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write('no hit here\n=== TIMEOUT ===\n')
        p = Path(f.name)
    try:
        assert classify_run_failure(p) == 'F1_timeout'
    finally:
        p.unlink(missing_ok=True)


def test_f1_capture_rc_124() -> None:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write('[HIT] x min_miss=0.1 m\n')
        p = Path(f.name)
    try:
        assert classify_run_failure(p, capture_rc=124) == 'F1_timeout'
    finally:
        p.unlink(missing_ok=True)


def test_f4_reassign() -> None:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write('reassign to interceptor_1\n')
        p = Path(f.name)
    try:
        assert classify_run_failure(p) == 'F4_assignment'
    finally:
        p.unlink(missing_ok=True)


def test_f5_hit_no_specials() -> None:
    text = (
        '[INFO] x: === Interceptor Selection ===\n'
        'selected: interceptor_0\n'
        '[HIT] interceptor_0  min_miss=0.5 m  hit_threshold = 1.0 m\n'
    )
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write(text)
        p = Path(f.name)
    try:
        assert classify_run_failure(p) == 'F5_unknown'
    finally:
        p.unlink(missing_ok=True)
