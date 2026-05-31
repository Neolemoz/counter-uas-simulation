"""Live runtime run capture helpers (Phase 6 Step 1)."""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.isolation import assert_capture_writable, repo_root_from, rt_sandbox_captures_dir

RUNTIME_CAPTURE_SCHEMA = "rt_runtime_run_capture_v1"
RUNTIME_CAPTURE_BANNER = "RT RUNTIME RUN CAPTURE - explanatory; not replay truth"
_RUNTIME_CAPTURE_REQUIRED_FIELDS = (
    "session_id",
    "capture_id",
    "started_utc",
    "stopped_utc",
    "timestamp",
    "entities",
    "telemetry_frames",
    "assignments",
    "lifecycle_transitions",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class RuntimeCaptureState:
    capture_id: str
    session_id: str
    started_utc: str
    telemetry_frames: list[dict[str, Any]] = field(default_factory=list)
    lifecycle_transitions: list[dict[str, Any]] = field(default_factory=list)


def begin_runtime_capture(session: Any) -> RuntimeCaptureState:
    state = RuntimeCaptureState(
        capture_id=str(uuid.uuid4()),
        session_id=session.session_id,
        started_utc=utc_now(),
    )
    session.runtime_capture = state
    return state


def runtime_capture_active(session: Any) -> bool:
    return getattr(session, "runtime_capture", None) is not None


def runtime_capture_status(session: Any) -> dict[str, Any]:
    state = getattr(session, "runtime_capture", None)
    world = getattr(session, "world", None)
    entities_count = 0
    if world is not None:
        entities_count = int(world.registry.count())
    if state is None:
        return {
            "capture_active": False,
            "capture_status": "inactive",
            "capture_id": None,
            "started_utc": None,
            "frames_count": 0,
            "entities_count": entities_count,
        }
    return {
        "capture_active": True,
        "capture_status": "active",
        "capture_id": state.capture_id,
        "started_utc": state.started_utc,
        "frames_count": len(state.telemetry_frames),
        "entities_count": entities_count,
    }


def record_runtime_capture_frame(
    session: Any,
    channel: str,
    payload: dict[str, Any],
) -> None:
    state = getattr(session, "runtime_capture", None)
    if state is None:
        return
    frame = {
        "timestamp_utc": utc_now(),
        "channel": channel,
        "payload": dict(payload),
    }
    state.telemetry_frames.append(frame)
    if channel == "lifecycle_state":
        state.lifecycle_transitions.append(frame)


def validate_runtime_capture_artifact(artifact: Any) -> dict[str, Any]:
    missing: list[str] = []
    type_errors: list[str] = []
    if not isinstance(artifact, dict):
        return {
            "valid": False,
            "schema": None,
            "missing": list(_RUNTIME_CAPTURE_REQUIRED_FIELDS),
            "type_errors": ["artifact must be object"],
        }

    for field_name in _RUNTIME_CAPTURE_REQUIRED_FIELDS:
        if field_name not in artifact:
            missing.append(field_name)

    for field_name in ("session_id", "capture_id", "started_utc", "stopped_utc", "timestamp"):
        if field_name in artifact and not isinstance(artifact[field_name], str):
            type_errors.append(f"{field_name} must be string")
    for field_name in ("entities", "telemetry_frames", "lifecycle_transitions"):
        if field_name in artifact and not isinstance(artifact[field_name], list):
            type_errors.append(f"{field_name} must be list")
    if "assignments" in artifact and not isinstance(artifact["assignments"], dict):
        type_errors.append("assignments must be object")

    return {
        "valid": not missing and not type_errors,
        "schema": artifact.get("schema"),
        "missing": missing,
        "type_errors": type_errors,
    }


def validate_runtime_capture_file(path: Path) -> dict[str, Any]:
    try:
        artifact = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            "valid": False,
            "schema": None,
            "missing": list(_RUNTIME_CAPTURE_REQUIRED_FIELDS),
            "type_errors": [str(exc)],
        }
    report = validate_runtime_capture_artifact(artifact)
    report["artifact_ref"] = path.as_posix()
    return report


def list_runtime_captures(repo_root: Path | None = None) -> list[dict[str, Any]]:
    root = repo_root or repo_root_from()
    captures_dir = rt_sandbox_captures_dir(root)
    captures: list[dict[str, Any]] = []
    for artifact_path in sorted(captures_dir.glob("*/runtime_run.json")):
        try:
            artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            report = validate_runtime_capture_file(artifact_path)
            captures.append(
                {
                    "artifact_ref": artifact_path.as_posix(),
                    "capture_id": artifact_path.parent.name,
                    "session_id": None,
                    "started_utc": None,
                    "stopped_utc": None,
                    "timestamp": None,
                    "valid": False,
                    "validation": report,
                }
            )
            continue
        report = validate_runtime_capture_artifact(artifact)
        captures.append(
            {
                "artifact_ref": artifact_path.as_posix(),
                "capture_id": artifact.get("capture_id") or artifact_path.parent.name,
                "session_id": artifact.get("session_id"),
                "started_utc": artifact.get("started_utc"),
                "stopped_utc": artifact.get("stopped_utc"),
                "timestamp": artifact.get("timestamp"),
                "valid": bool(report.get("valid")),
                "validation": report,
            }
        )
    return sorted(
        captures,
        key=lambda item: (str(item.get("stopped_utc") or ""), str(item.get("capture_id") or "")),
    )


def latest_runtime_capture(repo_root: Path | None = None) -> dict[str, Any] | None:
    captures = list_runtime_captures(repo_root)
    if not captures:
        return None
    return captures[-1]


def _audit_lifecycle_transitions(audit_path: Path | None, started_utc: str) -> list[dict[str, Any]]:
    if audit_path is None or not audit_path.exists():
        return []
    data = json.loads(audit_path.read_text(encoding="utf-8"))
    out: list[dict[str, Any]] = []
    for entry in data.get("entries", []):
        ts = str(entry.get("timestamp_utc") or "")
        if ts and ts < started_utc:
            continue
        detail = entry.get("detail") or {}
        command_type = str(entry.get("command_type") or "")
        if command_type in {
            "start_session",
            "pause_session",
            "resume",
            "stop_session",
            "reset_session",
            "runtime_crashed",
            "start_capture",
            "stop_capture",
        } or detail.get("state"):
            out.append(
                {
                    "timestamp_utc": ts,
                    "command_type": command_type,
                    "result": entry.get("result"),
                    "state": detail.get("state"),
                }
            )
    return out


def finalize_runtime_capture(
    session: Any,
    *,
    repo_root: Path,
    audit_path: Path | None,
) -> tuple[dict[str, Any], Path]:
    state = getattr(session, "runtime_capture", None)
    if state is None:
        raise RuntimeError("runtime capture not active")
    stopped_utc = utc_now()
    staging_dir = rt_sandbox_captures_dir(repo_root) / state.capture_id
    staging_dir.mkdir(parents=True, exist_ok=True)
    assert_capture_writable(staging_dir, repo_root)

    world_snapshot = session.world.snapshot().to_dict() if session.world else {}
    telemetry_transitions = [dict(frame) for frame in state.lifecycle_transitions]
    audit_transitions = _audit_lifecycle_transitions(audit_path, state.started_utc)
    artifact: dict[str, Any] = {
        "schema": RUNTIME_CAPTURE_SCHEMA,
        "capture_id": state.capture_id,
        "session_id": state.session_id,
        "started_utc": state.started_utc,
        "stopped_utc": stopped_utc,
        "timestamp": stopped_utc,
        "entities": world_snapshot.get("entities", []),
        "telemetry_frames": list(state.telemetry_frames),
        "assignments": dict(getattr(session, "live_assignments", {}) or {}),
        "lifecycle_transitions": telemetry_transitions + audit_transitions,
        "governance_banner": RUNTIME_CAPTURE_BANNER,
    }
    if world_snapshot.get("revision") is not None:
        artifact["world_revision"] = world_snapshot.get("revision")

    validation = validate_runtime_capture_artifact(artifact)
    if not validation["valid"]:
        raise RuntimeError(f"invalid runtime capture artifact: {validation}")

    artifact_path = staging_dir / "runtime_run.json"
    assert_capture_writable(artifact_path, repo_root)
    artifact_path.write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    session.runtime_capture = None
    return artifact, artifact_path
