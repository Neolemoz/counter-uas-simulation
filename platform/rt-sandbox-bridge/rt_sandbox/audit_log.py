"""Append-only rt_session_audit_log_v1."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.audit_vocabulary import classify_event_kind
from rt_sandbox.isolation import assert_writable_path, rt_sandbox_runs_dir


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class AuditLog:
    def __init__(self, repo_root: Path | None = None) -> None:
        self._repo_root = repo_root
        base = rt_sandbox_runs_dir(repo_root)
        self._audit_dir = base / "audit"
        self._audit_dir.mkdir(parents=True, exist_ok=True)

    def path_for(self, session_id: str) -> Path:
        p = self._audit_dir / f"{session_id}.json"
        assert_writable_path(p, self._repo_root)
        return p

    def append(
        self,
        session_id: str,
        *,
        command_id: str | None,
        command_type: str,
        issued_by: str,
        result: str,
        detail: dict[str, Any] | None = None,
    ) -> None:
        path = self.path_for(session_id)
        if path.exists():
            data = json.loads(path.read_text(encoding="utf-8"))
        else:
            data = {"schema": "rt_session_audit_log_v1", "session_id": session_id, "entries": []}
        entry: dict[str, Any] = {
            "command_id": command_id,
            "session_id": session_id,
            "command_type": command_type,
            "event_kind": classify_event_kind(command_type),
            "issued_by": issued_by,
            "result": result,
            "timestamp_utc": _utc_now(),
        }
        if detail:
            entry["detail"] = detail
        data["entries"].append(entry)
        path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
