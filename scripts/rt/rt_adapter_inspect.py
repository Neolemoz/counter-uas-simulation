#!/usr/bin/env python3
"""Read-only RT runtime adapter inspection (maintainer/dev only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.audit_log import AuditLog  # noqa: E402
from rt_sandbox.isolation import repo_root_from  # noqa: E402
from rt_sandbox.ros_allowlist import allowed_session_topics, classify_topic  # noqa: E402

_ADAPTER_CMDS = frozenset(
    {
        "adapter_attach",
        "adapter_detach",
        "adapter_teardown",
        "adapter_health",
        "orphan_cleanup",
        "ros_allowlist_reject",
        "gazebo_launch_failed",
        "start_session",
    }
)


def cmd_check_topic(session_id: str, topic: str) -> int:
    err = classify_topic(session_id, topic)
    if err:
        print(f"REJECT {topic} ({err})")
        return 1
    print(f"ALLOW {topic}")
    return 0


def cmd_list_topics(session_id: str) -> int:
    for t in sorted(allowed_session_topics(session_id)):
        print(t)
    return 0


def cmd_show_audit(repo_root: Path, session_id: str) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = [
        e for e in data.get("entries", []) if e.get("command_type") in _ADAPTER_CMDS
    ]
    print(json.dumps({"session_id": session_id, "entries": entries}, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RT adapter inspect (read-only)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_topics = sub.add_parser("list-topics", help="List allowed session topics")
    p_topics.add_argument("session_id")

    p_check = sub.add_parser("check-topic", help="Dry-run allow-list check")
    p_check.add_argument("session_id")
    p_check.add_argument("topic")

    p_audit = sub.add_parser("show-audit", help="Adapter-related audit entries")
    p_audit.add_argument("session_id")
    p_audit.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root (default: auto-detect)",
    )

    args = parser.parse_args()
    if args.cmd == "list-topics":
        return cmd_list_topics(args.session_id)
    if args.cmd == "check-topic":
        return cmd_check_topic(args.session_id, args.topic)
    if args.cmd == "show-audit":
        root = args.repo_root or repo_root_from(Path.cwd())
        return cmd_show_audit(root, args.session_id)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
