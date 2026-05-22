#!/usr/bin/env python3
"""Verify offline replay corpus reproducibility (PLAT-SA-F1b)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]

CHECKS = [
    ("build_replay_corpus_index.py", ["--check"]),
    ("validate_replay_corpus.py", []),
    ("build_replay_corpus_drift_report.py", ["--check"]),
    ("export_research_bundle.py", ["--check"]),
    ("build_replay_corpus_release.py", ["--check"]),
    ("diff_replay_corpus_releases.py", ["--check"]),
    ("audit_replay_corpus_provenance.py", []),
    ("build_replay_corpus_evolution.py", ["--check"]),
    ("build_replay_corpus_publication.py", ["--check"]),
    ("export_replay_corpus_release.py", ["--check"]),
]


def _run(script: str, args: list[str]) -> None:
    cmd = [sys.executable, str(_EVAL / script), *args]
    subprocess.run(cmd, check=True, cwd=_REPO)


def main() -> None:
    ap = argparse.ArgumentParser(description="Verify replay corpus reproducibility")
    ap.add_argument("--write-diff", action="store_true", help="regenerate release diff artifact")
    args = ap.parse_args()

    for script, script_args in CHECKS:
        if script == "diff_replay_corpus_releases.py" and args.write_diff:
            _run(script, [])
            continue
        _run(script, list(script_args))

    print("verify replay corpus reproducibility OK")


if __name__ == "__main__":
    main()
