#!/usr/bin/env python3
"""Batch replay corpus regeneration orchestrator (PLAT-SA-F1b)."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
_REPO = Path(__file__).resolve().parents[2]
_AUDITS = _REPO / "fixtures/sa_r0/corpus_audits"

STEPS: list[tuple[str, str, list[str]]] = [
    ("gen_f1_corpus", "gen_f1_corpus_fixtures.py", []),
    ("drift_report", "build_replay_corpus_drift_report.py", []),
    ("release_diff", "diff_replay_corpus_releases.py", []),
]

E2_STEPS: list[tuple[str, str, list[str]]] = [
    ("sync_catalog", "sync_sa_catalog.py", []),
    ("gen_d3", "gen_d3_sweep_enrichment.py", []),
    ("gen_e1", "gen_e1_presentation_fixtures.py", []),
    ("gen_e2", "gen_e2_research_fixtures.py", []),
]


def _run_step(script: str, args: list[str], *, dry_run: bool) -> dict[str, Any]:
    cmd = [sys.executable, str(_EVAL / script), *args]
    t0 = time.monotonic()
    if dry_run:
        return {
            "step": script,
            "status": "dry_run",
            "duration_ms": 0,
            "command": " ".join(cmd),
        }
    try:
        subprocess.run(cmd, check=True, cwd=_REPO)
        status = "ok"
        hint = ""
    except subprocess.CalledProcessError as exc:
        status = "failed"
        hint = str(exc)
    return {
        "step": script,
        "status": status,
        "duration_ms": int((time.monotonic() - t0) * 1000),
        "hint": hint,
        "command": " ".join(cmd),
    }


def run_regen(
    *,
    dry_run: bool = False,
    with_e2: bool = False,
    only: str | None = None,
    from_step: str | None = None,
) -> list[dict[str, Any]]:
    plan = list(E2_STEPS if with_e2 else []) + list(STEPS)
    if only:
        plan = [s for s in plan if s[0] == only]
    if from_step:
        ids = [s[0] for s in plan]
        if from_step in ids:
            plan = plan[ids.index(from_step) :]
    results: list[dict[str, Any]] = []
    for step_id, script, args in plan:
        rec = _run_step(script, args, dry_run=dry_run)
        rec["step_id"] = step_id
        results.append(rec)
        if rec["status"] == "failed":
            break
    return results


def write_regen_report(results: list[dict[str, Any]]) -> None:
    _AUDITS.mkdir(parents=True, exist_ok=True)
    report = {
        "artifact_type": "regen_run_report_v1",
        "schema_version": "regen_run_report_v1",
        "steps": results,
    }
    (_AUDITS / "regen_run_report_v1.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Run replay corpus regen workflow")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--with-e2", action="store_true", help="include E2/D3/E1 upstream steps")
    ap.add_argument("--only", help="run single step id")
    ap.add_argument("--from-step", help="start at step id")
    args = ap.parse_args()

    results = run_regen(
        dry_run=args.dry_run,
        with_e2=args.with_e2,
        only=args.only,
        from_step=args.from_step,
    )
    if not args.dry_run:
        write_regen_report(results)

    for rec in results:
        print(f"{rec.get('step_id')}: {rec.get('status')} ({rec.get('duration_ms')}ms)")

    if any(r.get("status") == "failed" for r in results):
        raise SystemExit(1)
    print("run_replay_corpus_regen OK")


if __name__ == "__main__":
    main()
