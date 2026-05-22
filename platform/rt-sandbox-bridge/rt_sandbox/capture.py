"""Runtime capture primitives and RT-only staging artifacts (PLAT-RT-S5)."""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.export_boundary import (
    CAPTURE_GOVERNANCE_BANNER,
    CONVERSION_GOVERNANCE_BANNER,
    CONVERSION_STEPS,
    ORIGIN_RT_SANDBOX_CAPTURE,
)
from rt_sandbox.isolation import assert_capture_writable, rt_sandbox_captures_dir

SNAPSHOT_BANNER = "RT SNAPSHOT — non-authoritative; not replay truth"
REPORT_BANNER = "CAPTURE REPORT — explanatory; not operational assessment"

DEFAULT_MAX_BUNDLE_BYTES = 5 * 1024 * 1024
DEFAULT_MAX_STAGED_CAPTURES = 32


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _write_json(path: Path, data: dict[str, Any], repo_root: Path) -> None:
    assert_capture_writable(path, repo_root)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class CaptureBundleError(Exception):
    """Capture bundle validation or size failure."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(message)


@dataclass
class CaptureBundleResult:
    capture_candidate_id: str
    staging_dir: Path
    staging_refs: dict[str, str]
    candidate: dict[str, Any]


def count_staged_captures(repo_root: Path | None = None) -> int:
    root = rt_sandbox_captures_dir(repo_root)
    return sum(1 for p in root.iterdir() if p.is_dir())


def validate_capture_payload(payload: Any, repo_root: Path | None = None) -> str | None:
    """Return error_code if payload invalid."""
    if payload is None:
        return None
    if not isinstance(payload, dict):
        return "INVALID_STATE"
    ref = payload.get("scenario_pack_ref")
    if ref is None:
        return None
    if not isinstance(ref, str) or not ref:
        return "INVALID_STATE"
    if not ref.startswith("fixtures/scenarios/"):
        return "INVALID_STATE"
    from rt_sandbox.isolation import repo_root_from

    root = repo_root or repo_root_from()
    full = (root / ref).resolve()
    if not full.is_file():
        return "INVALID_STATE"
    return None


def _summarize_audit(audit_path: Path | None) -> tuple[list[dict[str, Any]], list[str], list[str]]:
    command_summary: list[dict[str, Any]] = []
    resource_events: list[str] = []
    failure_states: list[str] = []
    if audit_path is None or not audit_path.exists():
        return command_summary, resource_events, failure_states
    data = json.loads(audit_path.read_text(encoding="utf-8"))
    for entry in data.get("entries", []):
        command_summary.append(
            {
                "command_type": entry.get("command_type"),
                "result": entry.get("result"),
                "timestamp_utc": entry.get("timestamp_utc"),
            }
        )
        if entry.get("result") == "RESOURCE_LIMIT_EXCEEDED":
            resource_events.append(entry.get("command_type", "unknown"))
        cmd = entry.get("command_type", "")
        if cmd in {"runtime_crashed", "bridge_ready_timeout"}:
            failure_states.append(cmd)
        detail = entry.get("detail") or {}
        if detail.get("state") in {"failed", "runtime_crashed"}:
            failure_states.append(str(detail.get("state")))
    return command_summary, resource_events, list(dict.fromkeys(failure_states))


def build_telemetry_summary(
    telemetry_store: Any,
    session_id: str,
) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "schema": "rt_telemetry_capture_summary_v1",
        "session_id": session_id,
        "channels": [],
        "event_counts": {},
        "governance_banner": SNAPSHOT_BANNER,
    }
    by_session = getattr(telemetry_store, "_by_session", {})
    sub = by_session.get(session_id)
    if sub is None:
        return summary
    summary["channels"] = sorted(sub.channels)
    summary["subscription_id"] = sub.subscription_id
    summary["event_counts"] = {"buffered": len(sub.events)}
    if sub.events:
        last = sub.events[-1]
        summary["last_channel"] = last.channel
        if last.channel == "world_summary":
            summary["last_world_summary"] = last.payload
    return summary


def build_capture_bundle(
    *,
    repo_root: Path,
    session_id: str,
    world_snapshot: dict[str, Any] | None,
    audit_path: Path | None,
    telemetry_store: Any,
    payload: dict[str, Any] | None,
    max_bundle_bytes: int = DEFAULT_MAX_BUNDLE_BYTES,
    max_staged: int = DEFAULT_MAX_STAGED_CAPTURES,
    workflow_summary: dict[str, Any] | None = None,
    templates_applied: list[str] | None = None,
) -> CaptureBundleResult:
    if count_staged_captures(repo_root) >= max_staged:
        raise CaptureBundleError("RESOURCE_LIMIT_EXCEEDED", "max staged captures exceeded")

    capture_id = str(uuid.uuid4())
    staging_dir = rt_sandbox_captures_dir(repo_root) / capture_id
    staging_dir.mkdir(parents=True, exist_ok=True)
    assert_capture_writable(staging_dir, repo_root)

    scenario_pack_ref = None
    if payload and isinstance(payload, dict):
        scenario_pack_ref = payload.get("scenario_pack_ref")

    command_summary, resource_events, failure_states = _summarize_audit(audit_path)
    telemetry_summary = build_telemetry_summary(telemetry_store, session_id)

    snapshot_doc: dict[str, Any] = {
        "schema": "sandbox_session_snapshot_v1",
        "session_id": session_id,
        "snapshot_utc": _utc_now(),
        "entity_states": world_snapshot.get("entities", []) if world_snapshot else [],
        "governance_banner": SNAPSHOT_BANNER,
    }
    if world_snapshot:
        snapshot_doc["revision"] = world_snapshot.get("revision")
        snapshot_doc["bounds"] = world_snapshot.get("bounds")

    report_doc: dict[str, Any] = {
        "schema": "runtime_capture_report_v1",
        "capture_candidate_id": capture_id,
        "session_id": session_id,
        "command_summary": command_summary,
        "resource_limit_events": resource_events,
        "failure_states_observed": failure_states,
        "governance_banner": REPORT_BANNER,
    }
    if workflow_summary:
        report_doc["workflow_summary"] = workflow_summary
    if templates_applied:
        report_doc["templates_applied"] = list(templates_applied)

    audit_ref = audit_path.as_posix() if audit_path else None
    snapshot_path = staging_dir / "snapshot.json"
    report_path = staging_dir / "capture_report.json"
    telemetry_path = staging_dir / "telemetry_summary.json"

    _write_json(snapshot_path, snapshot_doc, repo_root)
    _write_json(report_path, report_doc, repo_root)
    _write_json(telemetry_path, telemetry_summary, repo_root)

    staging_refs = {
        "snapshot_ref": snapshot_path.as_posix(),
        "capture_report_ref": report_path.as_posix(),
        "telemetry_summary_ref": telemetry_path.as_posix(),
    }
    if audit_ref:
        staging_refs["audit_ref"] = audit_ref

    candidate: dict[str, Any] = {
        "schema": "rt_capture_candidate_v1",
        "capture_candidate_id": capture_id,
        "session_id": session_id,
        "ephemeral_session_ref": session_id,
        "origin": ORIGIN_RT_SANDBOX_CAPTURE,
        "capture_utc": _utc_now(),
        "approval_status": "pending",
        "governance_banner": CAPTURE_GOVERNANCE_BANNER,
        "staging_refs": staging_refs,
    }
    if scenario_pack_ref:
        candidate["scenario_pack_ref"] = scenario_pack_ref

    candidate_path = staging_dir / "candidate.json"
    _write_json(candidate_path, candidate, repo_root)
    staging_refs["candidate_ref"] = candidate_path.as_posix()

    total_size = sum(
        f.stat().st_size for f in staging_dir.iterdir() if f.is_file()
    )
    if total_size > max_bundle_bytes:
        for f in staging_dir.iterdir():
            if f.is_file():
                f.unlink()
        staging_dir.rmdir()
        raise CaptureBundleError("RESOURCE_LIMIT_EXCEEDED", "capture bundle exceeds size cap")

    return CaptureBundleResult(
        capture_candidate_id=capture_id,
        staging_dir=staging_dir,
        staging_refs=staging_refs,
        candidate=candidate,
    )


def validate_capture_candidate(path: Path) -> list[str]:
    """Return list of validation errors (empty if ok)."""
    errors: list[str] = []
    if not path.exists():
        return ["candidate file not found"]
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "rt_capture_candidate_v1":
        errors.append("invalid schema")
    for key in ("capture_candidate_id", "session_id", "origin", "approval_status"):
        if not data.get(key):
            errors.append(f"missing {key}")
    if data.get("origin") != ORIGIN_RT_SANDBOX_CAPTURE:
        errors.append("invalid origin")
    if data.get("approval_status") not in {"pending", "approved", "rejected"}:
        errors.append("invalid approval_status")
    return errors


def write_approval_record(
    staging_dir: Path,
    *,
    approved_by: str,
    intent: str = "replay_research_import",
    repo_root: Path | None = None,
) -> dict[str, Any]:
    candidate_path = staging_dir / "candidate.json"
    errors = validate_capture_candidate(candidate_path)
    if errors:
        raise CaptureBundleError("INVALID_STATE", "; ".join(errors))
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    capture_id = candidate["capture_candidate_id"]
    approval: dict[str, Any] = {
        "schema": "rt_capture_approval_v1",
        "capture_candidate_id": capture_id,
        "approved_by": approved_by,
        "approved_utc": _utc_now(),
        "intent": intent,
        "governance_banner": "MAINTAINER APPROVAL — not automatic promotion",
    }
    from rt_sandbox.isolation import repo_root_from

    root = repo_root or repo_root_from(staging_dir)
    approval_path = staging_dir / "approval.json"
    _write_json(approval_path, approval, root)
    candidate["approval_status"] = "approved"
    _write_json(candidate_path, candidate, root)
    return approval


def write_conversion_manifest(
    staging_dir: Path,
    *,
    scenario_pack_ref: str | None = None,
    log_path: str | None = None,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    candidate_path = staging_dir / "candidate.json"
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    if candidate.get("approval_status") != "approved":
        raise CaptureBundleError("INVALID_STATE", "approval required before conversion manifest")
    capture_id = candidate["capture_candidate_id"]
    manifest: dict[str, Any] = {
        "schema": "runtime_to_replay_conversion_v1",
        "capture_candidate_id": capture_id,
        "ephemeral_session_ref": candidate.get("ephemeral_session_ref") or candidate.get("session_id"),
        "conversion_steps": list(CONVERSION_STEPS),
        "origin": ORIGIN_RT_SANDBOX_CAPTURE,
        "governance_banner": CONVERSION_GOVERNANCE_BANNER,
        "staging_refs": candidate.get("staging_refs", {}),
    }
    if scenario_pack_ref or candidate.get("scenario_pack_ref"):
        manifest["scenario_pack_ref"] = scenario_pack_ref or candidate.get("scenario_pack_ref")
    if log_path:
        manifest["log_path"] = log_path
    from rt_sandbox.isolation import repo_root_from

    root = repo_root or repo_root_from(staging_dir)
    manifest_path = staging_dir / "conversion.json"
    _write_json(manifest_path, manifest, root)
    return manifest


def list_staged_capture_ids(repo_root: Path | None = None) -> list[str]:
    root = rt_sandbox_captures_dir(repo_root)
    return sorted(p.name for p in root.iterdir() if p.is_dir() and (p / "candidate.json").exists())
