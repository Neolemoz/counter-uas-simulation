#!/usr/bin/env python3
"""Orchestrate F1d evolution, publication, and release export fixtures."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]


def _run(script: str, *args: str) -> None:
    cmd = [sys.executable, str(_EVAL / script), *args]
    subprocess.run(cmd, check=True, cwd=_REPO)


def main() -> None:
    _run("build_replay_corpus_index.py")
    _run("validate_replay_corpus.py")
    _run("build_replay_corpus_evolution.py")
    _run("build_replay_corpus_publication.py")
    _run("build_replay_corpus_release.py", "--refresh-index")
    _run("build_replay_corpus_drift_report.py")
    _run("diff_replay_corpus_releases.py")
    _run("export_replay_corpus_release.py")
    _run("build_replay_corpus_evolution.py")
    print("gen_f1d_publication_fixtures OK")


if __name__ == "__main__":
    main()
