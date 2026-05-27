"""Append-only export-boundary audit log."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.audit_vocabulary import EVENT_KIND_EXPORT
from rt_sandbox.isolation import assert_writable_path, rt_sandbox_runs_dir


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class ExportAuditLog:
    def __init__(self, repo_root: Path | None = None) -> None:
        self._repo_root = repo_root
        base = rt_sandbox_runs_dir(repo_root)
        self._dir = base / "export_audit"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path = self._dir / "export_boundary.jsonl"
        assert_writable_path(self._path, repo_root)

    @property
    def path(self) -> Path:
        return self._path

    def append(
        self,
        event_type: str,
        *,
        capture_candidate_id: str | None = None,
        session_id: str | None = None,
        result: str,
        detail: dict[str, Any] | None = None,
    ) -> None:
        entry: dict[str, Any] = {
            "schema": "rt_export_boundary_audit_v1",
            "event_type": event_type,
            "event_kind": EVENT_KIND_EXPORT,
            "timestamp_utc": _utc_now(),
            "result": result,
        }
        if capture_candidate_id:
            entry["capture_candidate_id"] = capture_candidate_id
        if session_id:
            entry["session_id"] = session_id
        if detail:
            entry["detail"] = detail
        with self._path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry, sort_keys=True) + "\n")
