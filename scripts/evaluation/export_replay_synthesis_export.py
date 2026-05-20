#!/usr/bin/env python3
"""Orchestrate full E2 export pipeline."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    subprocess.run([sys.executable, str(_EVAL / "gen_e2_research_fixtures.py")], check=True, cwd=_REPO)
    print("replay synthesis export OK")


if __name__ == "__main__":
    main()
