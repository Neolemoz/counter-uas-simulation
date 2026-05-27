#!/usr/bin/env python3
"""Maintainer handoff review for RT capture → SA import (no automatic import)."""

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
from rt_sandbox.isolation import repo_root_from  # noqa: E402
from rt_sandbox.sa_handoff import (  # noqa: E402
    append_handoff_event,
    capture_staging_dir,
    check_handoff_preconditions,
    sa_handoff_dir,
    write_handoff_review_v1,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RT→SA handoff review (maintainer gate; no SA import)"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    for name in ("ready", "reviewed", "reject", "defer"):
        p = sub.add_parser(name)
        p.add_argument("capture_id", help="capture_candidate_id")
        p.add_argument("--reviewer", default="maintainer_cli")
        p.add_argument("--notes", default=None)
        p.add_argument("--reason", default=None, help="reject reason (reject only)")
        p.add_argument("--dry-run", action="store_true")
        p.add_argument("--repo-root", type=Path, default=None)

    args = parser.parse_args()
    repo_root = args.repo_root or repo_root_from()
    staging = capture_staging_dir(repo_root, args.capture_id)
    if not staging.is_dir():
        print(f"Staging not found: {staging}", file=sys.stderr)
        return 1

    handoff_path = sa_handoff_dir(repo_root, args.capture_id)
    session_id = None
    cand_path = staging / "candidate.json"
    if cand_path.exists():
        session_id = json.loads(cand_path.read_text(encoding="utf-8")).get("session_id")

    try:
        if args.command == "ready":
            errors = check_handoff_preconditions(staging)
            if errors:
                print(f"Not ready: {'; '.join(errors)}", file=sys.stderr)
                return 1
            if args.dry_run:
                print(json.dumps({"dry_run": True, "capture_id": args.capture_id}, indent=2))
                return 0
            append_handoff_event(
                repo_root,
                "handoff_ready",
                capture_candidate_id=args.capture_id,
                session_id=session_id,
            )
            print(f"handoff_ready recorded for {args.capture_id}")
            return 0

        if args.command == "reviewed":
            errors = check_handoff_preconditions(staging)
            if errors:
                print(f"Review blocked: {'; '.join(errors)}", file=sys.stderr)
                return 1
            if args.dry_run:
                print(json.dumps({"dry_run": True, "decision": "ready_for_approval"}, indent=2))
                return 0
            review = write_handoff_review_v1(
                staging,
                capture_id=args.capture_id,
                decision="ready_for_approval",
                reviewer=args.reviewer,
                notes=args.notes,
                repo_root=repo_root,
                also_handoff_dir=handoff_path,
            )
            append_handoff_event(
                repo_root,
                "handoff_reviewed",
                capture_candidate_id=args.capture_id,
                session_id=session_id,
                detail={"reviewer": args.reviewer},
            )
            print(json.dumps(review, indent=2))
            return 0

        if args.command == "reject":
            notes = args.reason or args.notes or "handoff rejected by maintainer"
            if args.dry_run:
                print(json.dumps({"dry_run": True, "decision": "rejected"}, indent=2))
                return 0
            review = write_handoff_review_v1(
                staging,
                capture_id=args.capture_id,
                decision="rejected",
                reviewer=args.reviewer,
                notes=notes,
                repo_root=repo_root,
                also_handoff_dir=handoff_path,
            )
            append_handoff_event(
                repo_root,
                "handoff_rejected",
                capture_candidate_id=args.capture_id,
                session_id=session_id,
                detail={"notes": notes},
            )
            print(json.dumps(review, indent=2))
            return 0

        if args.command == "defer":
            notes = args.notes or "import deferred by maintainer"
            if args.dry_run:
                print(json.dumps({"dry_run": True, "decision": "deferred"}, indent=2))
                return 0
            review = write_handoff_review_v1(
                staging,
                capture_id=args.capture_id,
                decision="deferred",
                reviewer=args.reviewer,
                notes=notes,
                repo_root=repo_root,
                also_handoff_dir=handoff_path,
            )
            append_handoff_event(
                repo_root,
                "handoff_import_deferred",
                capture_candidate_id=args.capture_id,
                session_id=session_id,
                detail={"notes": notes},
            )
            print(json.dumps(review, indent=2))
            return 0

    except CaptureBundleError as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        return 1

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
