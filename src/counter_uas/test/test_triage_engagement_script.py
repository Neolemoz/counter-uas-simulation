"""Smoke: triage_engagement.sh is valid bash (explosion-chain section greps logs)."""

from __future__ import annotations

import subprocess
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]


def test_triage_engagement_shellcheck_syntax() -> None:
    script = _REPO_ROOT / 'scripts' / 'triage_engagement.sh'
    assert script.is_file(), script
    r = subprocess.run(
        ['bash', '-n', str(script)],
        cwd=str(_REPO_ROOT),
        check=False,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, r.stderr
