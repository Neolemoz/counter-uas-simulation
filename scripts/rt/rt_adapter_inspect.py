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

# Audit-log filter vocabulary — not RUNTIME_SUBCOMMANDS (see rt_runtime_subcommand_registry_v1.md).
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
        "sync_update",
        "sync_stale",
        "sync_mismatch",
        "adapter_feedback_lost",
        "mock_inject_drift",
        "telemetry_update",
        "telemetry_stale",
        "telemetry_feedback_lost",
        "telemetry_buffer_trim",
    }
)

_TELEMETRY_CMDS = frozenset(
    {
        "telemetry_update",
        "telemetry_stale",
        "telemetry_feedback_lost",
        "telemetry_buffer_trim",
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


def cmd_sync_status(repo_root: Path, session_id: str, *, live: bool = False) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    sync_types = {
        "sync_update",
        "sync_stale",
        "sync_mismatch",
        "sync_lag_observed",
        "sync_missing_feedback_entity",
        "adapter_feedback_lost",
    }
    entries = [
        e for e in data.get("entries", []) if e.get("command_type") in sync_types
    ]
    print(
        json.dumps(
            {
                "session_id": session_id,
                "live_detail": live,
                "sync_entries": entries,
                "count": len(entries),
            },
            indent=2,
        )
    )
    return 0


def cmd_world_health(repo_root: Path, session_id: str) -> int:
    """Read-only world/sync health summary from audit (G6 maintainer)."""
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = list(data.get("entries") or [])
    last_spawn = next(
        (e for e in reversed(entries) if e.get("command_type") == "spawn_entity"),
        None,
    )
    sync_events = [
        e.get("command_type")
        for e in entries
        if e.get("command_type") in {"sync_stale", "sync_mismatch", "adapter_feedback_lost"}
    ]
    print(
        json.dumps(
            {
                "session_id": session_id,
                "last_spawn_ok": last_spawn.get("result") == "OK" if last_spawn else None,
                "sync_event_count": len(sync_events),
                "sync_events_tail": sync_events[-5:],
            },
            indent=2,
        )
    )
    return 0


def cmd_telemetry_status(repo_root: Path, session_id: str) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = [
        e for e in data.get("entries", []) if e.get("command_type") in _TELEMETRY_CMDS
    ]
    print(
        json.dumps(
            {"session_id": session_id, "telemetry_entries": entries, "count": len(entries)},
            indent=2,
        )
    )
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

    p_sync = sub.add_parser("sync-status", help="Pose sync audit entries (G3)")
    p_sync.add_argument("session_id")
    p_sync.add_argument(
        "--live",
        action="store_true",
        help="Include live-mode sync detail flag in output (G6)",
    )
    p_sync.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root (default: auto-detect)",
    )

    p_world = sub.add_parser("world-health", help="World/sync health summary (G6)")
    p_world.add_argument("session_id")
    p_world.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root (default: auto-detect)",
    )

    p_tel = sub.add_parser("telemetry-status", help="Telemetry bridge audit entries (G4)")
    p_tel.add_argument("session_id")
    p_tel.add_argument(
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
    if args.cmd == "sync-status":
        root = args.repo_root or repo_root_from(Path.cwd())
        return cmd_sync_status(root, args.session_id, live=getattr(args, "live", False))
    if args.cmd == "world-health":
        root = args.repo_root or repo_root_from(Path.cwd())
        return cmd_world_health(root, args.session_id)
    if args.cmd == "telemetry-status":
        root = args.repo_root or repo_root_from(Path.cwd())
        return cmd_telemetry_status(root, args.session_id)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
