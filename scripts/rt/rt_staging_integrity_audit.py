#!/usr/bin/env python3
"""Read-only RT capture staging integrity audit (maintainer only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.isolation import repo_root_from, rt_sandbox_captures_dir  # noqa: E402


def audit_staging(repo_root: Path) -> dict[str, list[str]]:
    """Return issue buckets: missing_candidate, empty_dir, unreadable_candidate."""
    root = rt_sandbox_captures_dir(repo_root)
    issues: dict[str, list[str]] = {
        "missing_candidate": [],
        "empty_dir": [],
        "unreadable_candidate": [],
    }
    if not root.is_dir():
        return issues
    for entry in sorted(root.iterdir()):
        if not entry.is_dir():
            continue
        capture_id = entry.name
        cand = entry / "candidate.json"
        if not any(entry.iterdir()):
            issues["empty_dir"].append(capture_id)
            continue
        if not cand.is_file():
            issues["missing_candidate"].append(capture_id)
            continue
        try:
            json.loads(cand.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            issues["unreadable_candidate"].append(capture_id)
    return issues


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only scan of runs/rt_sandbox/captures staging integrity.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root (default: auto-detect from script location).",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit machine-readable JSON report.",
    )
    args = parser.parse_args()
    repo_root = args.repo_root or repo_root_from(_REPO)
    issues = audit_staging(repo_root)
    total = sum(len(v) for v in issues.values())
    if args.json:
        print(json.dumps({"repo_root": str(repo_root), "issues": issues, "total": total}, indent=2))
    else:
        print(f"RT staging integrity audit — repo={repo_root}")
        for kind, ids in issues.items():
            if not ids:
                continue
            print(f"\n{kind} ({len(ids)}):")
            for cid in ids:
                print(f"  - {cid}")
        if total == 0:
            print("\nNo staging integrity issues found.")
        else:
            print(f"\nTotal issues: {total}")
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main())
