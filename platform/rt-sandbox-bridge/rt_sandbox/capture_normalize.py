"""Runtime capture normalization (PLAT-RT-G5, PLAT-RT-R2e).

Revision vocabulary: docs/evaluation/rt_revision_vocabulary_v1.md
Authority model: docs/evaluation/rt_authority_model_v1.md
Capture pose cognition: docs/evaluation/rt_capture_pose_cognition_v1.md
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from rt_sandbox.authority_labels import (
    AUTHORITY_COMMAND,
    AUTHORITY_EXPLANATORY_SYNC,
    AUTHORITY_EXPLANATORY_TELEMETRY,
    AUTHORITY_REPLAY_BOUNDARY,
    authority_model_legend,
)
from rt_sandbox.capture_fidelity_coupling import (
    assess_fidelity_pose_block,
    fidelity_to_manifest_block,
)
from rt_sandbox.capture_pose_cognition import (
    assess_capture_pose_cognition,
    cognition_to_manifest_block,
)
from rt_sandbox.export_boundary import (
    NORMALIZATION_GOVERNANCE_BANNER,
    ORIGIN_RT_SANDBOX_CAPTURE,
    VALIDATION_GOVERNANCE_BANNER,
    validate_normalized_manifest,
)
from rt_sandbox.tactical_capture_annex import validate_tactical_annex
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.isolation import assert_capture_writable, repo_root_from

PROVENANCE_BANNER = "RUNTIME PROVENANCE — explanatory; not lineage authority"


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _write_json(path: Path, data: dict[str, Any], repo_root: Path) -> None:
    assert_capture_writable(path, repo_root)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _canonical_json(obj: Any) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"))


def _hash_raw_inputs(
    snapshot: dict[str, Any],
    report: dict[str, Any],
    telemetry_summary: dict[str, Any],
    runtime_context: dict[str, Any],
    tactical_annex: dict[str, Any] | None = None,
) -> str:
    payload = {
        "snapshot": snapshot,
        "report": report,
        "telemetry_summary": telemetry_summary,
        "runtime_context": runtime_context,
    }
    if tactical_annex is not None:
        payload["tactical_annex"] = tactical_annex
    return hashlib.sha256(_canonical_json(payload).encode("utf-8")).hexdigest()


class NormalizationError(Exception):
    def __init__(self, code: str, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(message)


@dataclass
class NormalizationContext:
    """Snapshot of runtime state at capture time (before teardown)."""

    session_state: str
    enable_gazebo_adapter: bool
    adapter_mode: str
    adapter_attached: bool
    pose_sync_summary: dict[str, Any] | None = None
    telemetry_mirror_summary: dict[str, Any] | None = None
    telemetry_channel_history: list[dict[str, Any]] = field(default_factory=list)
    workflow_summary: dict[str, Any] | None = None
    templates_applied: list[str] | None = None
    tactical_annex: dict[str, Any] | None = None
    tactical_annex_ref: str | None = None
    enable_fidelity_coupling: bool = False
    fidelity_truth_snapshot: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        out = {
            "session_state": self.session_state,
            "enable_gazebo_adapter": self.enable_gazebo_adapter,
            "adapter_mode": self.adapter_mode,
            "adapter_attached": self.adapter_attached,
            "enable_fidelity_coupling": self.enable_fidelity_coupling,
            "pose_sync_summary": self.pose_sync_summary,
            "telemetry_mirror_summary": self.telemetry_mirror_summary,
            "telemetry_channel_history": self.telemetry_channel_history,
            "workflow_summary": self.workflow_summary,
            "templates_applied": self.templates_applied,
            "fidelity_truth_snapshot": self.fidelity_truth_snapshot,
        }
        if self.tactical_annex is not None:
            out["tactical_annex_present"] = True
        return out


@dataclass
class NormalizationResult:
    capture_candidate_id: str
    conversion_revision: int
    normalization_utc: str
    staging_refs: dict[str, str]
    input_content_hash: str
    pose_assessment: Any | None = None
    fidelity_assessment: Any | None = None


def extract_telemetry_channel_history(
    telemetry_store: Any,
    session_id: str,
    *,
    max_events: int = 64,
    channels: frozenset[str] | None = None,
) -> list[dict[str, Any]]:
    """Bounded pose/telemetry history from subscription ring."""
    interest = channels or frozenset({"world_summary", "entity_pose_mirror"})
    by_session = getattr(telemetry_store, "_by_session", {})
    sub = by_session.get(session_id)
    if sub is None:
        return []
    out: list[dict[str, Any]] = []
    for ev in sub.events:
        if ev.channel not in interest:
            continue
        out.append(
            {
                "channel": ev.channel,
                "timestamp_utc": ev.timestamp_utc,
                "payload": ev.payload,
            }
        )
    return out[-max_events:]


def build_entity_pose_history(
    entity_states: list[dict[str, Any]],
    pose_sync_summary: dict[str, Any] | None,
    telemetry_mirror_summary: dict[str, Any] | None,
) -> list[dict[str, Any]]:
    feedback_by_id: dict[str, dict[str, Any]] = {}
    if pose_sync_summary:
        for ent in pose_sync_summary.get("feedback_entities") or []:
            eid = str(ent.get("entity_id", ""))
            if eid:
                feedback_by_id[eid] = ent

    mirror_by_id: dict[str, dict[str, Any]] = {}
    if telemetry_mirror_summary:
        for ent in (telemetry_mirror_summary.get("entity_pose_mirror") or {}).get(
            "entities"
        ) or []:
            eid = str(ent.get("entity_id", ""))
            if eid:
                mirror_by_id[eid] = ent

    history: list[dict[str, Any]] = []
    for state in entity_states:
        eid = str(state.get("entity_id", ""))
        if not eid:
            continue
        record: dict[str, Any] = {
            "entity_id": eid,
            "entity_type": state.get("entity_type"),
            "command_pose": dict(state.get("pose") or {}),
            "source": "bridge",
            "authority_label": AUTHORITY_COMMAND,
        }
        if state.get("revision") is not None:
            record["registry_revision"] = state.get("revision")
        fb = feedback_by_id.get(eid)
        if fb:
            record["feedback_pose"] = fb.get("feedback_pose")
            record["sim_entity_ref"] = fb.get("sim_entity_ref")
            record["sync_revision"] = fb.get("sync_revision")
            record["feedback_authority_label"] = AUTHORITY_EXPLANATORY_SYNC
            if fb.get("drift_m") is not None:
                record["drift_m"] = fb.get("drift_m")
        mir = mirror_by_id.get(eid)
        if mir:
            record["telemetry_mirror_pose"] = dict(mir.get("pose") or {})
            record["telemetry_source"] = "telemetry_mirror"
            record["telemetry_mirror_authority_label"] = AUTHORITY_EXPLANATORY_TELEMETRY
        history.append(record)
    return history


def _redact_staging_refs(refs: dict[str, str], staging_dir: Path) -> dict[str, str]:
    """Keep only refs under the capture staging directory."""
    prefix = staging_dir.as_posix()
    out: dict[str, str] = {}
    for key, ref in refs.items():
        if key == "audit_ref":
            continue
        if ref.startswith(prefix) or not ref.startswith("/"):
            if not ref.startswith("runs/rt_sandbox/audit"):
                out[key] = ref
    return out


def _build_provenance(
    *,
    capture_id: str,
    candidate: dict[str, Any],
    staging_dir: Path,
    ctx: NormalizationContext,
    input_hash: str,
    entity_pose_history: list[dict[str, Any]],
) -> dict[str, Any]:
    sim_mapping: dict[str, str | None] = {}
    for ent in entity_pose_history:
        eid = str(ent.get("entity_id", ""))
        if not eid:
            continue
        sim_mapping[eid] = ent.get("sim_entity_ref")

    raw_refs = candidate.get("staging_refs") or {}
    source_refs = _redact_staging_refs(raw_refs, staging_dir)
    if ctx.tactical_annex_ref:
        annex_ref = ctx.tactical_annex_ref
        if annex_ref.startswith(staging_dir.as_posix()) or not annex_ref.startswith("/"):
            if "runs/rt_sandbox/audit" not in annex_ref:
                source_refs["tactical_annex_ref"] = annex_ref
    return {
        "schema": "rt_capture_provenance_v1",
        "capture_candidate_id": capture_id,
        "origin": ORIGIN_RT_SANDBOX_CAPTURE,
        "normalization_utc": _utc_now(),
        "adapter_attached": ctx.adapter_attached,
        "adapter_mode": ctx.adapter_mode,
        "enable_gazebo_adapter": ctx.enable_gazebo_adapter,
        "ephemeral_session_ref": candidate.get("ephemeral_session_ref"),
        "input_content_hash": input_hash,
        "source_artifact_refs": source_refs,
        "entity_sim_mapping": sim_mapping,
        "governance_banner": PROVENANCE_BANNER,
    }


def _next_conversion_revision(
    staging_dir: Path,
    input_hash: str,
) -> int:
    prov_path = staging_dir / "provenance.json"
    if prov_path.exists():
        prev = json.loads(prov_path.read_text(encoding="utf-8"))
        if prev.get("input_content_hash") == input_hash:
            norm_path = staging_dir / "normalized_manifest.json"
            if norm_path.exists():
                norm = json.loads(norm_path.read_text(encoding="utf-8"))
                return int(norm.get("conversion_revision") or 1)
    norm_path = staging_dir / "normalized_manifest.json"
    if norm_path.exists():
        norm = json.loads(norm_path.read_text(encoding="utf-8"))
        prev_hash_path = staging_dir / "provenance.json"
        if prev_hash_path.exists():
            prev = json.loads(prev_hash_path.read_text(encoding="utf-8"))
            if prev.get("input_content_hash") != input_hash:
                return int(norm.get("conversion_revision") or 0) + 1
        return int(norm.get("conversion_revision") or 0) + 1
    return 1


def _load_raw_artifacts(staging_dir: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    for name in ("candidate.json", "snapshot.json", "capture_report.json", "telemetry_summary.json"):
        if not (staging_dir / name).exists():
            raise NormalizationError("INVALID_STATE", f"missing {name}")
    candidate = json.loads((staging_dir / "candidate.json").read_text(encoding="utf-8"))
    snapshot = json.loads((staging_dir / "snapshot.json").read_text(encoding="utf-8"))
    report = json.loads((staging_dir / "capture_report.json").read_text(encoding="utf-8"))
    telemetry_summary = json.loads(
        (staging_dir / "telemetry_summary.json").read_text(encoding="utf-8")
    )
    return candidate, snapshot, report, telemetry_summary


def normalize_capture_bundle(
    staging_dir: Path,
    *,
    runtime_context: NormalizationContext,
    repo_root: Path | None = None,
    config: GovernanceConfig | None = None,
) -> NormalizationResult:
    root = repo_root or repo_root_from(staging_dir)
    candidate, snapshot, report, telemetry_summary = _load_raw_artifacts(staging_dir)
    capture_id = candidate["capture_candidate_id"]
    session_id = candidate.get("session_id", "")

    ctx_dict = runtime_context.to_dict()
    input_hash = _hash_raw_inputs(
        snapshot,
        report,
        telemetry_summary,
        ctx_dict,
        runtime_context.tactical_annex,
    )
    conversion_revision = _next_conversion_revision(staging_dir, input_hash)
    norm_utc = _utc_now()

    entity_states = list(snapshot.get("entity_states") or [])
    entity_pose_history = build_entity_pose_history(
        entity_states,
        runtime_context.pose_sync_summary,
        runtime_context.telemetry_mirror_summary,
    )

    pose_assessment = assess_capture_pose_cognition(
        entity_pose_history,
        runtime_context.pose_sync_summary,
        adapter_attached=runtime_context.adapter_attached,
    )

    lifecycle_summary: dict[str, Any] = {
        "session_state_at_capture": runtime_context.session_state,
        "command_count": len(report.get("command_summary") or []),
        "failure_states_observed": report.get("failure_states_observed") or [],
        "resource_limit_events": report.get("resource_limit_events") or [],
    }
    if runtime_context.workflow_summary:
        lifecycle_summary["workflow_summary"] = runtime_context.workflow_summary
    if runtime_context.templates_applied:
        lifecycle_summary["templates_applied"] = list(runtime_context.templates_applied)

    normalized: dict[str, Any] = {
        "schema": "rt_normalized_capture_v1",
        "capture_candidate_id": capture_id,
        "ephemeral_session_ref": candidate.get("ephemeral_session_ref") or session_id,
        "origin": ORIGIN_RT_SANDBOX_CAPTURE,
        "conversion_revision": conversion_revision,
        "normalization_utc": norm_utc,
        "entity_pose_history": entity_pose_history,
        "lifecycle_summary": lifecycle_summary,
        "telemetry_summary_snapshot": {
            "channels": telemetry_summary.get("channels") or [],
            "event_counts": telemetry_summary.get("event_counts") or {},
            "channel_history": runtime_context.telemetry_channel_history,
        },
        "sync_health_summary": runtime_context.pose_sync_summary,
        "telemetry_health_summary": runtime_context.telemetry_mirror_summary,
        "capture_timestamps": {
            "capture_utc": candidate.get("capture_utc"),
            "snapshot_utc": snapshot.get("snapshot_utc"),
            "snapshot_revision": snapshot.get("revision"),
        },
        "authority_model": {
            "legend": authority_model_legend(),
            "manifest_authority_label": AUTHORITY_REPLAY_BOUNDARY,
        },
        "capture_pose_cognition": cognition_to_manifest_block(pose_assessment),
        "governance_banner": NORMALIZATION_GOVERNANCE_BANNER,
    }
    fidelity_assessment = assess_fidelity_pose_block(
        entity_pose_history,
        runtime_context.fidelity_truth_snapshot,
        enable_fidelity_coupling=runtime_context.enable_fidelity_coupling,
        adapter_attached=runtime_context.adapter_attached,
    )
    if fidelity_assessment is not None:
        normalized["fidelity_pose_block"] = fidelity_to_manifest_block(fidelity_assessment)
    if runtime_context.tactical_annex is not None:
        annex_err = validate_tactical_annex(runtime_context.tactical_annex)
        if annex_err:
            raise NormalizationError("INVALID_STATE", annex_err)
        if runtime_context.tactical_annex.get("capture_candidate_id") != capture_id:
            raise NormalizationError(
                "INVALID_STATE",
                "tactical annex capture_candidate_id mismatch",
            )
        if runtime_context.tactical_annex.get("ephemeral_session_ref") != session_id:
            raise NormalizationError(
                "INVALID_STATE",
                "tactical annex session ref mismatch",
            )
        normalized["tactical_annex"] = runtime_context.tactical_annex

    provenance = _build_provenance(
        capture_id=capture_id,
        candidate=candidate,
        staging_dir=staging_dir,
        ctx=runtime_context,
        input_hash=input_hash,
        entity_pose_history=entity_pose_history,
    )

    manifest_err = validate_normalized_manifest(normalized)
    boundary_checks: list[dict[str, Any]] = [
        {"check": "normalized_manifest_schema", "pass": manifest_err is None},
    ]
    if manifest_err:
        boundary_checks.append({"check": "detail", "message": manifest_err})

    prov_checks = [
        {"check": "no_external_audit_ref", "pass": "audit_ref" not in provenance.get("source_artifact_refs", {})},
        {
            "check": "session_id_not_parent_ref",
            "pass": normalized.get("parent_ref") != session_id,
        },
    ]
    if runtime_context.tactical_annex is not None:
        prov_checks.append(
            {
                "check": "tactical_annex_schema",
                "pass": validate_tactical_annex(runtime_context.tactical_annex) is None,
            }
        )
        prov_checks.append(
            {
                "check": "tactical_annex_session_scoped",
                "pass": runtime_context.tactical_annex.get("ephemeral_session_ref")
                == session_id,
            }
        )
        prov_checks.append(
            {
                "check": "no_authority_escalation",
                "pass": not runtime_context.tactical_annex.get("authoritative_parent_ref"),
            }
        )
    boundary_checks.extend(prov_checks)

    valid = manifest_err is None and all(c.get("pass") for c in boundary_checks)

    validation_doc: dict[str, Any] = {
        "schema": "rt_normalization_validation_v1",
        "capture_candidate_id": capture_id,
        "valid": valid,
        "checks": boundary_checks,
        "boundary_lint": {"export_boundary": "rt_sa_export_boundary_v1"},
        "governance_banner": VALIDATION_GOVERNANCE_BANNER,
        "validated_utc": norm_utc,
    }

    if not valid:
        raise NormalizationError("INVALID_STATE", manifest_err or "normalization validation failed")

    norm_path = staging_dir / "normalized_manifest.json"
    prov_path = staging_dir / "provenance.json"
    val_path = staging_dir / "normalization_validation.json"

    _write_json(norm_path, normalized, root)
    _write_json(prov_path, provenance, root)
    _write_json(val_path, validation_doc, root)

    staging_refs = dict(candidate.get("staging_refs") or {})
    staging_refs["normalized_manifest_ref"] = norm_path.as_posix()
    staging_refs["provenance_ref"] = prov_path.as_posix()
    staging_refs["validation_ref"] = val_path.as_posix()
    if runtime_context.tactical_annex_ref:
        staging_refs["tactical_annex_ref"] = runtime_context.tactical_annex_ref
    candidate["staging_refs"] = staging_refs
    candidate["normalization_status"] = "normalized"
    candidate["conversion_revision"] = conversion_revision
    candidate["normalization_utc"] = norm_utc
    _write_json(staging_dir / "candidate.json", candidate, root)

    return NormalizationResult(
        capture_candidate_id=capture_id,
        conversion_revision=conversion_revision,
        normalization_utc=norm_utc,
        staging_refs=staging_refs,
        input_content_hash=input_hash,
        pose_assessment=pose_assessment,
        fidelity_assessment=fidelity_assessment,
    )


def mark_normalization_rejected(staging_dir: Path, *, reason: str, repo_root: Path | None = None) -> None:
    root = repo_root or repo_root_from(staging_dir)
    cand_path = staging_dir / "candidate.json"
    if not cand_path.exists():
        return
    candidate = json.loads(cand_path.read_text(encoding="utf-8"))
    candidate["normalization_status"] = "rejected"
    candidate["normalization_reject_reason"] = reason
    _write_json(cand_path, candidate, root)


def validate_normalized_capture(staging_dir: Path) -> list[str]:
    errors: list[str] = []
    norm_path = staging_dir / "normalized_manifest.json"
    if not norm_path.exists():
        return ["normalized_manifest.json not found"]
    norm = json.loads(norm_path.read_text(encoding="utf-8"))
    err = validate_normalized_manifest(norm)
    if err:
        errors.append(err)
    prov_path = staging_dir / "provenance.json"
    if prov_path.exists():
        prov = json.loads(prov_path.read_text(encoding="utf-8"))
        refs = prov.get("source_artifact_refs") or {}
        for ref in refs.values():
            if "runs/rt_sandbox/audit" in str(ref):
                errors.append("provenance contains external audit_ref")
    cand_path = staging_dir / "candidate.json"
    if cand_path.exists():
        cand = json.loads(cand_path.read_text(encoding="utf-8"))
        if cand.get("normalization_status") != "normalized":
            errors.append(f"normalization_status is {cand.get('normalization_status')}")
    annex_path = staging_dir / "tactical_annex.json"
    if annex_path.exists():
        annex = json.loads(annex_path.read_text(encoding="utf-8"))
        annex_err = validate_tactical_annex(annex)
        if annex_err:
            errors.append(annex_err)
        if norm_path.exists():
            norm_annex = norm.get("tactical_annex")
            if norm_annex is None:
                errors.append("normalized manifest missing tactical_annex sidecar")
            elif norm_annex.get("capture_candidate_id") != norm.get("capture_candidate_id"):
                errors.append("tactical annex capture id mismatch in normalized manifest")
    return errors


def build_normalization_context_from_session(
    session: Any,
    config: GovernanceConfig,
    telemetry_store: Any,
) -> NormalizationContext:
    from rt_sandbox.runtime_handle import runtime_is_adapter
    from rt_sandbox.telemetry_bridge import telemetry_bridge_active

    pose_sync_summary = None
    if session.pose_sync is not None and session.world is not None:
        pose_sync_summary = session.pose_sync.summary(session.world.revision)

    telemetry_mirror_summary = None
    if session.telemetry_mirror is not None:
        telemetry_mirror_summary = session.telemetry_mirror.summary()

    adapter_attached = (
        config.enable_gazebo_adapter and runtime_is_adapter(session.runtime)
    )
    adapter_mode = config.adapter_mode
    if adapter_attached:
        health = session.runtime.health_payload()
        adapter_mode = str(health.get("adapter_mode") or config.adapter_mode)

    history = extract_telemetry_channel_history(
        telemetry_store,
        session.session_id,
        max_events=config.telemetry_ring_buffer_size,
    )

    workflow_summary = session.workflow.to_dict() if session.workflow else None
    templates = list(session.templates_applied) if session.templates_applied else None

    fidelity_truth_snapshot = None
    if config.enable_fidelity_coupling and session.telemetry_mirror is not None:
        fidelity_truth_snapshot = session.telemetry_mirror.summary().get("fidelity_truth")

    return NormalizationContext(
        session_state=session.state.value,
        enable_gazebo_adapter=config.enable_gazebo_adapter,
        adapter_mode=adapter_mode,
        adapter_attached=adapter_attached and telemetry_bridge_active(session, config)
        or adapter_attached,
        pose_sync_summary=pose_sync_summary,
        telemetry_mirror_summary=telemetry_mirror_summary,
        telemetry_channel_history=history,
        workflow_summary=workflow_summary,
        templates_applied=templates,
        enable_fidelity_coupling=config.enable_fidelity_coupling,
        fidelity_truth_snapshot=fidelity_truth_snapshot,
    )


def staging_bundle_size(staging_dir: Path) -> int:
    return sum(f.stat().st_size for f in staging_dir.iterdir() if f.is_file())
