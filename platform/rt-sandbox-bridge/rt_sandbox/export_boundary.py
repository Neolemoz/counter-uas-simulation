"""RT↔SA export boundary enforcement (rt_sa_export_boundary_v1)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from rt_sandbox.isolation import assert_sa_path_blocked, assert_writable_path, repo_root_from
from rt_sandbox.tactical_capture_annex import validate_tactical_annex

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
NORMALIZATION_GOVERNANCE_BANNER = (
    "NORMALIZED CAPTURE — replay-ready RT artifact; not SA replay authority"
)
VALIDATION_GOVERNANCE_BANNER = (
    "NORMALIZATION VALIDATION — boundary lint only; not import approval"
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


def validate_normalized_manifest(manifest: dict[str, Any]) -> str | None:
    """Return error message if normalized manifest violates export boundary."""
    if manifest.get("schema") != "rt_normalized_capture_v1":
        return "invalid schema"
    if manifest.get("origin") != ORIGIN_RT_SANDBOX_CAPTURE:
        return "origin must be rt_sandbox_capture_v1"
    if not manifest.get("capture_candidate_id"):
        return "capture_candidate_id required"
    parent_ref = manifest.get("parent_ref")
    session_id = manifest.get("session_id")
    if parent_ref and session_id and parent_ref == session_id:
        return "session_id must not be authoritative parent_ref"
    if manifest.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on normalized manifest"
    ephemeral = manifest.get("ephemeral_session_ref")
    if parent_ref and session_id and ephemeral and parent_ref == session_id:
        return "session_id must not be lineage parent_ref"
    annex = manifest.get("tactical_annex")
    if annex is not None:
        annex_err = validate_tactical_annex(annex)
        if annex_err:
            return annex_err
    return None


def validate_conversion_manifest(manifest: dict[str, Any]) -> str | None:
    """Return error message if manifest violates export boundary, else None."""
    if manifest.get("schema") != "runtime_to_replay_conversion_v1":
        return "invalid schema"
    if manifest.get("origin") != ORIGIN_RT_SANDBOX_CAPTURE:
        return "origin must be rt_sandbox_capture_v1"
    steps = manifest.get("conversion_steps")
    if not isinstance(steps, list) or not steps:
        return "conversion_steps required"
    parent_ref = manifest.get("parent_ref")
    session_id = manifest.get("session_id")
    if parent_ref and session_id and parent_ref == session_id:
        return "session_id must not be authoritative parent_ref"
    if manifest.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on RT conversion manifest"
    ephemeral = manifest.get("ephemeral_session_ref")
    if parent_ref and session_id and ephemeral and parent_ref == session_id:
        return "session_id must not be lineage parent_ref"
    staging_refs = manifest.get("staging_refs") or {}
    if manifest.get("requires_normalization", True):
        if not staging_refs.get("normalized_manifest_ref"):
            return "normalized_manifest_ref required in staging_refs"
    return None


def reject_auto_sa_import(reason: str) -> None:
    """Explicit block for automatic SA replay import from RT bridge."""
    raise ExportBoundaryError(f"automatic SA import forbidden: {reason}")


def assert_maintainer_corpus_write_allowed(target_path: Path, repo_root: Path | None = None) -> None:
    """Allow maintainer import commit only under fixtures/sa_r0/."""
    root = repo_root or repo_root_from()
    resolved = target_path.resolve()
    allowed = (root / "fixtures" / "sa_r0").resolve()
    if not allowed.exists():
        allowed.mkdir(parents=True, exist_ok=True)
    if not resolved.is_relative_to(allowed):
        raise ExportBoundaryError(
            f"maintainer corpus import limited to fixtures/sa_r0/ (got {resolved})"
        )


def validate_sa_import_record(record: dict[str, Any]) -> str | None:
    """Return error message if import record violates lineage rules."""
    if record.get("schema") != "rt_sa_import_record_v1":
        return "invalid schema"
    if not record.get("corpus_ref"):
        return "corpus_ref required"
    if not record.get("bundle_path"):
        return "bundle_path required"
    parent_ref = record.get("parent_ref")
    session_id = record.get("session_id")
    if parent_ref and session_id and parent_ref == session_id:
        return "session_id must not be authoritative parent_ref"
    if record.get("authoritative_parent_ref"):
        return "authoritative_parent_ref forbidden on import record"
    rt_ref = record.get("rt_capture_ref") or {}
    if isinstance(rt_ref, dict) and rt_ref.get("session_id") == parent_ref:
        return "rt_capture_ref must not elevate session_id to parent_ref"
    return None


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
