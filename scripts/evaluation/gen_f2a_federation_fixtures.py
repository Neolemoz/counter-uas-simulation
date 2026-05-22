#!/usr/bin/env python3
"""Orchestrate PLAT-SA-F2A federation fixture generation."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_PY = sys.executable
_EVAL = _REPO / "scripts" / "evaluation"


def _run(script: str, *args: str) -> None:
    cmd = [_PY, str(_EVAL / script), *args]
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=_REPO)


def main() -> None:
    _run("build_replay_federation_index.py", "--write")
    _run("audit_replay_federation_integrity.py", "--write")
    _run("build_replay_federation_reproducibility.py", "--write")
    _run("build_replay_federation_snapshot.py", "--write")
    _run("audit_federation_recovery_continuity.py", "--write")
    print("gen_f2a_federation_fixtures: OK")


if __name__ == "__main__":
    main()
