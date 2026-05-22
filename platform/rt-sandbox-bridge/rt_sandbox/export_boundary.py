"""RT↔SA export boundary enforcement (rt_sa_export_boundary_v1)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from rt_sandbox.isolation import assert_sa_path_blocked, assert_writable_path, repo_root_from

BLOCKED_WRITE_PREFIXES = (
    "fixtures/sa_r0",
    "fixtures/orchestration",
    "fixtures/scenarios",
    "replay_federation",
    "platform/sa-r0-viewer",
)

CAPTURE_GOVERNANCE_BANNER = (
    "CAPTURE CANDIDATE — requires validation before replay import; not operational state"
)
CONVERSION_GOVERNANCE_BANNER = (
    "CONVERSION MANIFEST — maintainer pipeline only; not replay authority"
)

ORIGIN_RT_SANDBOX_CAPTURE = "rt_sandbox_capture_v1"

CONVERSION_STEPS = (
    "validate_scenario_pack",
    "replay_observability",
    "replay_sa_bundle_pack",
    "governance_lint",
)


class ExportBoundaryError(PermissionError):
    """Raised when an export or import path violates RT↔SA boundary."""


def assert_rt_export_allowed(target_path: Path, repo_root: Path | None = None) -> None:
    """Deny-by-default: only runs/rt_sandbox/ writable; block SA/federation targets."""
    root = repo_root or repo_root_from()
    try:
        assert_writable_path(target_path, root)
    except PermissionError as exc:
        raise ExportBoundaryError(str(exc)) from exc
    try:
        assert_sa_path_blocked(target_path, root)
    except PermissionError as exc:
        raise ExportBoundaryError(str(exc)) from exc


def validate_conversion_manifest(manifest: dict[str, Any]) -> str | None:
    """Return error message if manifest violates export boundary, else None."""
    if manifest.get("schema") != "runtime_to_replay_conversion_v1":
        return "invalid schema"
    if manifest.get("origin") != ORIGIN_RT_SANDBOX_CAPTURE:
        return "origin must be rt_sandbox_capture_v1"
    steps = manifest.get("conversion_steps")
    if not isinstance(steps, list) or not steps:
        return "conversion_steps required"
    if manifest.get("parent_ref") == manifest.get("session_id"):
        return "session_id must not be authoritative parent_ref"
    if manifest.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on RT conversion manifest"
    ephemeral = manifest.get("ephemeral_session_ref")
    session_id = manifest.get("session_id")
    if ephemeral and session_id and manifest.get("parent_ref") == session_id:
        return "session_id must not be lineage parent_ref"
    return None


def reject_auto_sa_import(reason: str) -> None:
    """Explicit block for automatic SA replay import from RT bridge."""
    raise ExportBoundaryError(f"automatic SA import forbidden: {reason}")


def block_federation_orchestration_command(command_type: str) -> bool:
    """True if command must be rejected (federation/orchestration)."""
    low = command_type.lower()
    if low in {
        "corpus_promote",
        "federation_register",
        "publish_to_collection",
        "launch_queue",
        "run_experiment_queue",
    }:
        return True
    for sub in ("corpus_", "federation_", "orchestration", "replay_sa"):
        if sub in low:
            return True
    return False
