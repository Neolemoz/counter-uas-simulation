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
            "capture_session",
            "federation_register",
        ],
        help="bridge command",
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--session-id", default=None)
    args = parser.parse_args()
    try:
        out = send_command(
            args.command,
            url=args.url,
            session_id=args.session_id,
        )
    except urllib.error.URLError as exc:
        print(json.dumps({"ok": False, "error_code": "BRIDGE_DISCONNECTED", "message": str(exc)}))
        return 1
    print(json.dumps(out, indent=2, sort_keys=True))
    return 0 if out.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
