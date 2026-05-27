#!/usr/bin/env python3
"""Maintainer RT→SA manual import bridge (prepare, pipeline steps, corpus commit)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.capture import CaptureBundleError  # noqa: E402
from rt_sandbox.export_boundary import CONVERSION_STEPS  # noqa: E402
from rt_sandbox.isolation import repo_root_from  # noqa: E402
from rt_sandbox.sa_handoff import (  # noqa: E402
    append_handoff_event,
    capture_staging_dir,
    commit_bundle_to_corpus,
    handoff_status_summary,
    is_handoff_blocked,
    require_approved_for_import,
    run_conversion_step,
    sa_handoff_dir,
    write_handoff_manifest,
)


def cmd_prepare(repo_root: Path, capture_id: str, dry_run: bool) -> int:
    staging = capture_staging_dir(repo_root, capture_id)
    if is_handoff_blocked(staging):
        print("Handoff blocked (rejected or deferred)", file=sys.stderr)
        return 1
    _approval, conversion = require_approved_for_import(staging)
    if dry_run:
        print(json.dumps({"dry_run": True, "capture_id": capture_id}, indent=2))
        return 0
    manifest = write_handoff_manifest(repo_root, capture_id, staging, conversion)
    session_id = json.loads((staging / "candidate.json").read_text(encoding="utf-8")).get(
        "session_id"
    )
    append_handoff_event(
        repo_root,
        "handoff_import_prepared",
        capture_candidate_id=capture_id,
        session_id=session_id,
    )
    print(json.dumps(manifest, indent=2))
    return 0


def cmd_status(repo_root: Path, capture_id: str) -> int:
    print(json.dumps(handoff_status_summary(repo_root, capture_id), indent=2))
    return 0


def cmd_run_step(
    repo_root: Path, capture_id: str, step: str, dry_run: bool
) -> int:
    if step not in CONVERSION_STEPS:
        print(f"Unknown step; expected one of {CONVERSION_STEPS}", file=sys.stderr)
        return 1
    staging = capture_staging_dir(repo_root, capture_id)
    if is_handoff_blocked(staging):
        print("Handoff blocked", file=sys.stderr)
        return 1
    handoff_path = sa_handoff_dir(repo_root, capture_id)
    manifest_path = handoff_path / "handoff_manifest.json"
    if not manifest_path.exists():
        print("Run prepare first", file=sys.stderr)
        return 1
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    result = run_conversion_step(
        step,
        repo_root=repo_root,
        handoff_manifest=manifest,
        handoff_path=handoff_path,
        dry_run=dry_run,
    )
    print(json.dumps(result, indent=2))
    return 0 if result.get("ok") else 1


def cmd_run_pipeline(repo_root: Path, capture_id: str, dry_run: bool) -> int:
    for step in CONVERSION_STEPS:
        rc = cmd_run_step(repo_root, capture_id, step, dry_run)
        if rc != 0:
            return rc
    return 0


def cmd_commit(
    repo_root: Path,
    capture_id: str,
    corpus_dest: Path,
    imported_by: str,
    dry_run: bool,
) -> int:
    staging = capture_staging_dir(repo_root, capture_id)
    if is_handoff_blocked(staging):
        print("Handoff blocked", file=sys.stderr)
        return 1
    try:
        record = commit_bundle_to_corpus(
            repo_root,
            capture_id,
            corpus_dest.resolve(),
            imported_by=imported_by,
            dry_run=dry_run,
        )
    except CaptureBundleError as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        return 1
    if dry_run:
        print(json.dumps(record, indent=2))
        return 0
    session_id = json.loads((staging / "candidate.json").read_text(encoding="utf-8")).get(
        "session_id"
    )
    append_handoff_event(
        repo_root,
        "handoff_import_committed",
        capture_candidate_id=capture_id,
        session_id=session_id,
        detail={"corpus_ref": record.get("corpus_ref")},
    )
    print(json.dumps(record, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RT→SA manual import bridge (maintainer only; no auto-import)"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    prep = sub.add_parser("prepare", help="Write sa_handoff manifest after approval")
    prep.add_argument("capture_id")
    prep.add_argument("--dry-run", action="store_true")

    stat = sub.add_parser("status", help="Handoff status summary")
    stat.add_argument("capture_id")

    step_p = sub.add_parser("run-step", help="Run one conversion step")
    step_p.add_argument("capture_id")
    step_p.add_argument("step", choices=list(CONVERSION_STEPS))
    step_p.add_argument("--dry-run", action="store_true")

    pipe = sub.add_parser("run-pipeline", help="Run all conversion steps in order")
    pipe.add_argument("capture_id")
    pipe.add_argument("--dry-run", action="store_true")

    commit_p = sub.add_parser("commit", help="Copy handoff bundle to corpus path")
    commit_p.add_argument("capture_id")
    commit_p.add_argument(
        "--corpus-dest",
        type=Path,
        required=True,
        help="Destination under fixtures/sa_r0/",
    )
    commit_p.add_argument("--imported-by", default="maintainer_cli")
    commit_p.add_argument("--dry-run", action="store_true")

    args = parser.parse_args()
    repo_root = repo_root_from()

    if args.command == "prepare":
        return cmd_prepare(repo_root, args.capture_id, args.dry_run)
    if args.command == "status":
        return cmd_status(repo_root, args.capture_id)
    if args.command == "run-step":
        return cmd_run_step(repo_root, args.capture_id, args.step, args.dry_run)
    if args.command == "run-pipeline":
        return cmd_run_pipeline(repo_root, args.capture_id, args.dry_run)
    if args.command == "commit":
        return cmd_commit(
            repo_root,
            args.capture_id,
            args.corpus_dest,
            args.imported_by,
            args.dry_run,
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
