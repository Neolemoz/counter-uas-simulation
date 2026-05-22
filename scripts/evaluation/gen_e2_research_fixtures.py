#!/usr/bin/env python3
"""Orchestrate E2 synthesis, linkage, figures, assets, and publication exports."""

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
    _run("build_cross_sweep_synthesis.py", "--all")
    _run("build_replay_linkage.py")
    _run("build_replay_cognition_rollup.py")
    _run("replay_viz_sweep_figures.py")
    _run("gen_presentation_assets.py")
    _run("export_presentation_pack.py", "--all", "--publication")
    _run("export_research_bundle.py", "--corpus", "sa_r0_corpus_r1", "--zip")
    print("gen_e2_research_fixtures OK")


if __name__ == "__main__":
    main()
