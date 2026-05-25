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
        f.write('no hit here\n')
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


def test_reassignment_inequality_diagnostic_is_not_assignment_failure() -> None:
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write('  Reassignment inequality: tti(best)+margin < tti(commit)  ->  false\n')
        p = Path(f.name)
    try:
        assert classify_run_failure(p) == 'F5_unknown'
    finally:
        p.unlink(missing_ok=True)


def test_hit_logs_are_not_failures_even_when_capture_times_out() -> None:
    text = (
        '[INFO] x: === Interceptor Selection ===\n'
        'selected: interceptor_0\n'
        '[HIT] interceptor_0  min_miss=0.5 m  hit_threshold = 1.0 m\n'
        '=== TIMEOUT ===\n'
    )
    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False, encoding='utf-8') as f:
        f.write(text)
        p = Path(f.name)
    try:
        assert classify_run_failure(p, capture_rc=124) == ''
    finally:
        p.unlink(missing_ok=True)
