#!/usr/bin/env python3
"""Aggregate summarize_run + selection_audit + optional meta JSON for evaluation matrices."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict as dc_asdict
from pathlib import Path

_SCRIPTS_DIR = Path(__file__).resolve().parent.parent  # scripts/
_EVAL_DIR = Path(__file__).resolve().parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
if str(_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(_EVAL_DIR))

import summarize_run  # noqa: E402
from classify_ambiguity_failure import classify_ambiguity_failure  # noqa: E402
from classify_realism_failure import classify_realism_failure  # noqa: E402
from realism_metrics import realism_metrics_summary  # noqa: E402
from selection_audit import selection_audit_summary  # noqa: E402


def evaluation_row(log_path: Path, *, meta_path: Path | None = None) -> dict:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    run_id = log_path.stem
    ss = summarize_run.parse_log(text, run_id=run_id)
    sa = selection_audit_summary(text)
    rm = realism_metrics_summary(text)
    meta: dict = {}
    if meta_path and meta_path.is_file():
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            meta = {"meta_error": "unreadable"}

    git_commit = meta.get("git_commit")
    dirty = meta.get("git_dirty")
    notes_v = meta.get("notes")

    row = dc_asdict(ss)
    row.update(sa)
    row.update(rm)
    row["ambiguity_failure_class"] = classify_ambiguity_failure(row)
    row["realism_failure_class"] = classify_realism_failure(row)
    row.update(
        {
            "git_commit": git_commit,
            "git_dirty": dirty,
            "notes_meta": notes_v,
        },
    )
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description="One JSON evaluation row combining summary + oracle audit.")
    ap.add_argument("log", type=Path)
    ap.add_argument("--meta", type=Path, default=None, help=".meta.json from run_capture")
    args = ap.parse_args()

    row = evaluation_row(Path(args.log), meta_path=args.meta)
    sys.stdout.write(json.dumps(row, sort_keys=True, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
