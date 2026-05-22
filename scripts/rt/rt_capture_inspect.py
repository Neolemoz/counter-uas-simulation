#!/usr/bin/env python3
"""Read-only RT capture staging inspection (dev/maintainer only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.capture import list_staged_capture_ids, validate_capture_candidate  # noqa: E402
from rt_sandbox.isolation import repo_root_from, rt_sandbox_captures_dir  # noqa: E402


def cmd_list(repo_root: Path) -> int:
    ids = list_staged_capture_ids(repo_root)
    if not ids:
        print("No staged captures.")
        return 0
    for cid in ids:
        cand_path = rt_sandbox_captures_dir(repo_root) / cid / "candidate.json"
        data = json.loads(cand_path.read_text(encoding="utf-8"))
        print(
            f"{cid}\tapproval={data.get('approval_status')}\t"
            f"session={data.get('ephemeral_session_ref', data.get('session_id'))}"
        )
    return 0


def cmd_show(repo_root: Path, capture_id: str) -> int:
    staging = rt_sandbox_captures_dir(repo_root) / capture_id
    cand = staging / "candidate.json"
    if not cand.exists():
        print(f"Capture not found: {capture_id}", file=sys.stderr)
        return 1
    print(json.dumps(json.loads(cand.read_text(encoding="utf-8")), indent=2))
    return 0


def cmd_validate(repo_root: Path, capture_id: str | None) -> int:
    root = rt_sandbox_captures_dir(repo_root)
    targets = [capture_id] if capture_id else list_staged_capture_ids(repo_root)
    if not targets:
        print("No captures to validate.")
        return 0
    failed = 0
    for cid in targets:
        errors = validate_capture_candidate(root / cid / "candidate.json")
        if errors:
            print(f"{cid}: FAIL — {', '.join(errors)}")
            failed += 1
        else:
            print(f"{cid}: OK")
    return 1 if failed else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect RT capture staging (read-only)")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="List staged capture candidate IDs")
    show_p = sub.add_parser("show", help="Show candidate.json for one capture")
    show_p.add_argument("capture_id")
    val_p = sub.add_parser("validate", help="Validate candidate schema(s)")
    val_p.add_argument("capture_id", nargs="?", default=None)
    args = parser.parse_args()
    repo_root = repo_root_from()
    if args.command == "list":
        return cmd_list(repo_root)
    if args.command == "show":
        return cmd_show(repo_root, args.capture_id)
    if args.command == "validate":
        return cmd_validate(repo_root, args.capture_id)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
