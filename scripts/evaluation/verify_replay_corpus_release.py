#!/usr/bin/env python3
"""Verify replay corpus release snapshot and archive (PLAT-SA-F1d)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_RELEASE = "sa_r0_corpus_r1_r1"


def _run(script: str, *args: str) -> None:
    cmd = [sys.executable, str(_EVAL / script), *args]
    subprocess.run(cmd, check=True, cwd=_REPO)


def main() -> None:
    ap = argparse.ArgumentParser(description="Verify replay corpus release")
    ap.add_argument("--release", default=_DEFAULT_RELEASE)
    args = ap.parse_args()

    _run("build_replay_corpus_release.py", "--check", "--release", args.release)
    _run("export_replay_corpus_release.py", "--check", "--release", args.release)
    print("verify replay corpus release OK")


if __name__ == "__main__":
    main()
