"""Read-only handoff mirror command handlers (PLAT-RT-SA2)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from rt_sandbox.capture_handoff_mirror import list_capture_handoff_status_response
from rt_sandbox.isolation import repo_root_from
from rt_sandbox.session_response import fail, ok


def handle_list_capture_handoff_status(
    base: dict[str, Any],
    payload: Any,
    *,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        return fail(base, "INVALID_STATE", "payload required")
    session_id = payload.get("session_id")
    if not isinstance(session_id, str) or not session_id.strip():
        return fail(base, "INVALID_STATE", "session_id required in payload")

    root = repo_root or repo_root_from(Path.cwd())
    mirror = list_capture_handoff_status_response(root, session_id.strip())
    resp = ok(base, state="handoff_mirror")
    resp.update(mirror)
    return resp
