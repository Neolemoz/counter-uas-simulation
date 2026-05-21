#!/usr/bin/env python3
"""Orchestrate F1a corpus index, validation, release snapshot, and research bundle."""

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
    _run("export_research_bundle.py", "--corpus", "sa_r0_corpus_r1")
    _run("build_replay_corpus_index.py")
    _run("build_replay_corpus_release.py")
    _run("build_replay_corpus_drift_report.py")
    _run("diff_replay_corpus_releases.py")
    _run("audit_replay_lineage.py")
    print("gen_f1_corpus_fixtures OK")


if __name__ == "__main__":
    main()
