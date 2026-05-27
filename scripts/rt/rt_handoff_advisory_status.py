#!/usr/bin/env python3
"""Read-only RT handoff advisory status (PLAT-RT-F6 P0)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.advisory_derive import derive_advisory_status_for_capture  # noqa: E402
from rt_sandbox.isolation import repo_root_from  # noqa: E402


def cmd_status(repo_root: Path, capture_id: str, *, as_json: bool) -> int:
    try:
        status = derive_advisory_status_for_capture(repo_root, capture_id)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if as_json:
        print(json.dumps(status, indent=2, sort_keys=True))
    else:
        state = status.get("advisory_state")
        label = status.get("advisory_state_label", "")
        blocked = status.get("blocked", False)
        print(f"{capture_id}\tadvisory={state}\t{label}\tblocked={blocked}")
        for reason in status.get("block_reasons") or []:
            print(f"  blocker: {reason}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only RT→SA handoff advisory status (maintainer only)."
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root (default: auto-detect from cwd)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    status_p = sub.add_parser("status", help="Derive advisory status for a capture")
    status_p.add_argument("capture_id", help="Capture candidate id")
    status_p.add_argument("--json", action="store_true", help="Emit full JSON document")

    args = parser.parse_args()
    repo_root = args.repo_root or repo_root_from(Path.cwd())

    if args.command == "status":
        return cmd_status(repo_root, args.capture_id, as_json=args.json)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
