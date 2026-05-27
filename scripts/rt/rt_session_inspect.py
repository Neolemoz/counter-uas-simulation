#!/usr/bin/env python3
"""Read-only RT multi-session inspection (maintainer/dev only — not RUNTIME_SUBCOMMAND)."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
_SCRIPTS_RT = _REPO / "scripts" / "rt"
for p in (_BRIDGE_PKG, _SCRIPTS_RT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rt_bridge_client import DEFAULT_URL, send_command  # noqa: E402
from rt_sandbox.audit_log import AuditLog  # noqa: E402
from rt_sandbox.isolation import repo_root_from  # noqa: E402

_POLL_HINT_CMDS = frozenset(
    {
        "subscribe_telemetry",
        "unsubscribe_telemetry",
        "telemetry_stale",
        "telemetry_update",
        "telemetry_feedback_lost",
    }
)


def _fetch_list_sessions(url: str) -> dict[str, Any]:
    try:
        out = send_command("list_sessions", url=url, issued_by="rt_session_inspect")
    except urllib.error.URLError as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "error_code": "BRIDGE_DISCONNECTED",
                    "message": str(exc),
                }
            ),
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    if not out.get("ok"):
        print(json.dumps(out, indent=2), file=sys.stderr)
        raise SystemExit(1)
    return out


def _session_row(listed: dict[str, Any], session_id: str) -> dict[str, Any] | None:
    for row in listed.get("sessions") or []:
        if row.get("session_id") == session_id:
            return row
    return None


def _audit_tail(repo_root: Path, session_id: str, tail: int) -> list[dict[str, Any]]:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = list(data.get("entries") or [])
    if tail <= 0:
        return entries
    return entries[-tail:]


def _poll_hints(entries: list[dict[str, Any]]) -> dict[str, Any]:
    hints: dict[str, Any] = {"poll_state": "ui_managed"}
    for cmd in _POLL_HINT_CMDS:
        matches = [e for e in entries if e.get("command_type") == cmd]
        if matches:
            hints[f"last_{cmd}"] = matches[-1].get("timestamp_utc")
    return hints


def cmd_list(url: str, *, as_json: bool) -> int:
    listed = _fetch_list_sessions(url)
    if as_json:
        print(json.dumps(listed, indent=2, sort_keys=True))
        return 0
    print(
        f"capacity={listed.get('capacity')} "
        f"non_terminal={listed.get('non_terminal_count')} "
        f"editing={listed.get('editing_session_id')}"
    )
    for row in listed.get("sessions") or []:
        edit = "*" if row.get("is_editing") else " "
        print(
            f"{edit} {row.get('session_id')}\t{row.get('state')}\t"
            f"entities={row.get('entity_count')}"
        )
    return 0


def cmd_summary(url: str, *, as_json: bool) -> int:
    listed = _fetch_list_sessions(url)
    payload = {
        "ok": True,
        "capacity": listed.get("capacity"),
        "non_terminal_count": listed.get("non_terminal_count"),
        "editing_session_id": listed.get("editing_session_id"),
        "session_count": len(listed.get("sessions") or []),
    }
    if as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"capacity: {payload['capacity']}")
        print(f"non_terminal_count: {payload['non_terminal_count']}")
        print(f"editing_session_id: {payload['editing_session_id']}")
        print(f"sessions_in_registry: {payload['session_count']}")
    return 0


def cmd_audit(
    repo_root: Path,
    session_id: str,
    *,
    tail: int,
    as_json: bool,
) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    before = path.read_text(encoding="utf-8")
    entries = _audit_tail(repo_root, session_id, tail)
    payload = {
        "session_id": session_id,
        "audit_path": str(path),
        "tail": tail,
        "entries": entries,
        "count": len(entries),
    }
    if path.read_text(encoding="utf-8") != before:
        print("Audit file mutated during read", file=sys.stderr)
        return 1
    if as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"Session {session_id} — audit tail ({len(entries)} entries)")
        for entry in entries:
            print(
                f"  {entry.get('timestamp_utc', '')}  "
                f"{entry.get('command_type', '')}  {entry.get('result', '')}"
            )
    return 0


def cmd_show(
    url: str,
    repo_root: Path,
    session_id: str,
    *,
    tail: int,
    as_json: bool,
) -> int:
    listed = _fetch_list_sessions(url)
    row = _session_row(listed, session_id)
    if row is None:
        print(f"Session not found in registry: {session_id}", file=sys.stderr)
        return 1
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    entries = _audit_tail(repo_root, session_id, tail)
    is_editing = bool(row.get("is_editing"))
    payload: dict[str, Any] = {
        "ok": True,
        "session_id": session_id,
        "summary": row,
        "lifecycle": row.get("state"),
        "registry_role": "editing" if is_editing else "non_editing",
        "ui_active_background_note": (
            "Workstation active/background tabs are UI-only; "
            "registry_role reflects editing_session_id lock."
        ),
        "polling": _poll_hints(entries),
        "audit_tail": entries,
        "background_active_state": {
            "bridge_editing_lock": is_editing,
            "entity_count": row.get("entity_count"),
        },
    }
    if as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"session_id: {session_id}")
        print(f"lifecycle: {row.get('state')}")
        print(f"registry_role: {payload['registry_role']}")
        print(f"entity_count: {row.get('entity_count')}")
        print(f"polling: {payload['polling']['poll_state']} (bridge does not track pull clocks)")
        for key, val in payload["polling"].items():
            if key != "poll_state":
                print(f"  audit hint {key}: {val}")
        print(f"audit tail ({len(entries)} entries):")
        for entry in entries:
            print(
                f"  {entry.get('timestamp_utc', '')}  "
                f"{entry.get('command_type', '')}  {entry.get('result', '')}"
            )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RT multi-session inspection (read-only; loopback list_sessions + audit files)",
    )
    parser.add_argument("--url", default=DEFAULT_URL, help="Bridge command URL")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root for audit files (default: auto-detect)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="List sessions via list_sessions")
    p_list.add_argument("--json", action="store_true")

    p_summary = sub.add_parser("summary", help="Registry capacity summary")
    p_summary.add_argument("--json", action="store_true")

    p_show = sub.add_parser("show", help="Session registry row + audit tail")
    p_show.add_argument("session_id")
    p_show.add_argument("--tail", type=int, default=10)
    p_show.add_argument("--json", action="store_true")

    p_audit = sub.add_parser("audit", help="Read-only audit log tail")
    p_audit.add_argument("session_id")
    p_audit.add_argument("--tail", type=int, default=10)
    p_audit.add_argument("--json", action="store_true")

    args = parser.parse_args()
    repo_root = args.repo_root or repo_root_from(_REPO)

    if args.command == "list":
        return cmd_list(args.url, as_json=args.json)
    if args.command == "summary":
        return cmd_summary(args.url, as_json=args.json)
    if args.command == "show":
        return cmd_show(
            args.url,
            repo_root,
            args.session_id,
            tail=args.tail,
            as_json=args.json,
        )
    if args.command == "audit":
        return cmd_audit(
            repo_root,
            args.session_id,
            tail=args.tail,
            as_json=args.json,
        )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
