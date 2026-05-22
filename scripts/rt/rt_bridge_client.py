#!/usr/bin/env python3
"""CLI harness for RT sandbox bridge (dev/test only — not SA viewer)."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path

DEFAULT_URL = "http://127.0.0.1:18765/v1/command"


def send_command(
    command_type: str,
    *,
    url: str,
    session_id: str | None = None,
    issued_by: str = "maintainer_cli",
    payload: dict | None = None,
) -> dict:
    body = {
        "schema": "rt_bridge_request_v1",
        "command_type": command_type,
        "command_id": str(uuid.uuid4()),
        "issued_by": issued_by,
        "authority_scope": "rt_sandbox_prototype",
    }
    if session_id:
        body["session_id"] = session_id
    if payload is not None:
        body["payload"] = payload
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description="RT bridge CLI client")
    parser.add_argument(
        "command",
        choices=[
            "start_session",
            "pause_session",
            "resume",
            "stop_session",
            "discard_session",
            "reset_session",
            "spawn_entity",
            "move_entity",
            "delete_entity",
            "subscribe_telemetry",
            "unsubscribe_telemetry",
            "capture_session",
            "list_runtime_templates",
            "apply_runtime_template",
            "start_workflow",
            "advance_workflow",
            "reset_workflow",
            "reload_workflow",
            "get_workflow_state",
            "federation_register",
        ],
        help="bridge command",
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--session-id", default=None)
    parser.add_argument(
        "--payload",
        default=None,
        help="JSON payload for entity/telemetry commands",
    )
    parser.add_argument("--payload-file", default=None, help="Path to JSON payload file")
    args = parser.parse_args()

    payload = None
    if args.payload_file:
        payload = json.loads(Path(args.payload_file).read_text(encoding="utf-8"))
    elif args.payload:
        payload = json.loads(args.payload)

    try:
        out = send_command(
            args.command,
            url=args.url,
            session_id=args.session_id,
            payload=payload,
        )
    except urllib.error.URLError as exc:
        print(json.dumps({"ok": False, "error_code": "BRIDGE_DISCONNECTED", "message": str(exc)}))
        return 1
    print(json.dumps(out, indent=2, sort_keys=True))
    return 0 if out.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
