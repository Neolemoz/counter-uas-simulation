#!/usr/bin/env python3
"""Read-only RT workflow and template inspection (dev/maintainer only)."""

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
from rt_sandbox.template_catalog import list_templates_metadata  # noqa: E402
from rt_sandbox.workflow import list_workflows_metadata  # noqa: E402

_WORKFLOW_CMDS = frozenset(
    {
        "start_workflow",
        "advance_workflow",
        "reset_workflow",
        "reload_workflow",
        "apply_runtime_template",
    }
)


def cmd_list_templates() -> int:
    for row in list_templates_metadata():
        print(
            f"{row['template_id']}\t{row['kind']}\t"
            f"entities={row['entity_count']}\t{row['description']}"
        )
    return 0


def cmd_list_workflows() -> int:
    for row in list_workflows_metadata():
        print(f"{row['workflow_id']}\tsteps={row['step_count']}\t{row['description']}")
    return 0


def cmd_show_audit(repo_root: Path, session_id: str) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = [
        e
        for e in data.get("entries", [])
        if e.get("command_type") in _WORKFLOW_CMDS
        or e.get("command_type") == "reset_session"
    ]
    print(json.dumps({"session_id": session_id, "entries": entries}, indent=2))
    return 0


def cmd_summary(repo_root: Path, session_id: str) -> int:
    path = AuditLog(repo_root).path_for(session_id)
    if not path.exists():
        print(f"Audit not found: {session_id}", file=sys.stderr)
        return 1
    data = json.loads(path.read_text(encoding="utf-8"))
    print(f"Session {session_id} — workflow/template timeline")
    for entry in data.get("entries", []):
        cmd = entry.get("command_type", "")
        if cmd not in _WORKFLOW_CMDS and cmd != "reset_session":
            continue
        ts = entry.get("timestamp_utc", "")
        detail = entry.get("detail") or {}
        wf = detail.get("workflow_id", "")
        tpl = detail.get("template_id", "")
        trans = detail.get("transition", "")
        step = detail.get("step_index", "")
        parts = [p for p in (wf, tpl, trans, str(step) if step != "" else "") if p]
        extra = " ".join(parts)
        print(f"  {ts}  {cmd}  {entry.get('result')}  {extra}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RT workflow/template inspection (read-only)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-templates")
    sub.add_parser("list-workflows")

    p_audit = sub.add_parser("show-audit")
    p_audit.add_argument("session_id")

    p_sum = sub.add_parser("summary")
    p_sum.add_argument("session_id")

    args = parser.parse_args()
    if args.command == "list-templates":
        return cmd_list_templates()
    if args.command == "list-workflows":
        return cmd_list_workflows()

    repo_root = repo_root_from(_REPO)
    if args.command == "show-audit":
        return cmd_show_audit(repo_root, args.session_id)
    if args.command == "summary":
        return cmd_summary(repo_root, args.session_id)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
