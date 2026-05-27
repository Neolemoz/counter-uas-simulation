"""Capture session command handler (PLAT-RT-R3a)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rt_sandbox.audit_log import AuditLog
from rt_sandbox.capture import CaptureBundleError, build_capture_bundle
from rt_sandbox.capture_normalize import (
    NormalizationError,
    build_normalization_context_from_session,
    mark_normalization_rejected,
    normalize_capture_bundle,
    staging_bundle_size,
    validate_normalized_capture,
)
from rt_sandbox.capture_fidelity_coupling import (
    append_fidelity_capture_audits,
    export_fidelity_capture_detail,
)
from rt_sandbox.capture_pose_cognition import (
    append_capture_pose_audits,
    export_pose_normalized_detail,
)
from rt_sandbox.tactical_capture_annex import (
    append_tactical_capture_audits,
    build_tactical_capture_annex_v1,
    write_tactical_annex_file,
)
from rt_sandbox.tactical_controller import TacticalController
from rt_sandbox.export_audit_log import ExportAuditLog
from rt_sandbox.export_boundary import CAPTURE_GOVERNANCE_BANNER
from rt_sandbox.governance import GovernanceConfig
from rt_sandbox.isolation import repo_root_from
from rt_sandbox.lifecycle import SessionState, can_transition
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.session_response import fail, ok
from rt_sandbox.session_teardown import TeardownDeps, teardown_capture_post_session
from rt_sandbox.telemetry_subscriptions import TelemetrySubscriptionStore


def capture_session(
    session: SessionRecord,
    base: dict[str, Any],
    *,
    config: GovernanceConfig,
    audit: AuditLog,
    export_audit: ExportAuditLog,
    telemetry_subs: TelemetrySubscriptionStore,
    repo_root: Path | None,
    command_id: str,
    issued_by: str,
    payload: Any,
) -> dict[str, Any]:
    if not can_transition(session.state, "capture_session"):
        export_audit.append(
            "capture_rejected",
            session_id=session.session_id,
            result="INVALID_STATE",
            detail={"state": session.state.value},
        )
        return fail(base, "INVALID_STATE", session.state.value)

    export_audit.append(
        "capture_requested",
        session_id=session.session_id,
        result="pending",
        detail={"issued_by": issued_by},
    )

    root = repo_root or repo_root_from()
    world_snapshot = session.world.snapshot().to_dict() if session.world else None
    audit_path = audit.path_for(session.session_id)

    workflow_summary = session.workflow.to_dict() if session.workflow else None
    templates_applied = list(session.templates_applied)

    try:
        bundle = build_capture_bundle(
            repo_root=root,
            session_id=session.session_id,
            world_snapshot=world_snapshot,
            audit_path=audit_path,
            telemetry_store=telemetry_subs,
            payload=payload if isinstance(payload, dict) else None,
            max_bundle_bytes=config.max_capture_bundle_bytes,
            max_staged=config.max_staged_captures,
            workflow_summary=workflow_summary,
            templates_applied=templates_applied,
        )
    except CaptureBundleError as exc:
        export_audit.append(
            "capture_rejected",
            session_id=session.session_id,
            result=exc.code,
            detail={"message": exc.message},
        )
        audit.append(
            session.session_id,
            command_id=command_id,
            command_type="capture_session",
            issued_by=issued_by,
            result=exc.code,
            detail={"message": exc.message},
        )
        return fail(base, exc.code, exc.message)

    tactical_annex: dict[str, Any] | None = None
    tactical_annex_ref: str | None = None
    tactical_buffer = None
    if session.tactical is not None and isinstance(session.tactical, TacticalController):
        tactical = session.tactical
        tactical_buffer = tactical.capture_buffer
        tactical_annex = build_tactical_capture_annex_v1(
            session_id=session.session_id,
            tactical=tactical,
            capture_candidate_id=bundle.capture_candidate_id,
        )
        if tactical_annex is not None:
            annex_path = write_tactical_annex_file(
                bundle.staging_dir,
                tactical_annex,
                repo_root=root,
            )
            tactical_annex_ref = annex_path.as_posix()
            cand_path = bundle.staging_dir / "candidate.json"
            candidate = json.loads(cand_path.read_text(encoding="utf-8"))
            staging_refs = dict(candidate.get("staging_refs") or {})
            staging_refs["tactical_annex_ref"] = tactical_annex_ref
            candidate["staging_refs"] = staging_refs
            from rt_sandbox.capture_normalize import _write_json

            _write_json(cand_path, candidate, root)
            bundle.staging_refs = staging_refs

    if config.enable_fidelity_coupling and session.telemetry_mirror is not None:
        ft = session.telemetry_mirror.fidelity_truth
        if ft is not None:
            from rt_sandbox.capture_normalize import _write_json

            ft_path = bundle.staging_dir / "fidelity_truth.json"
            _write_json(ft_path, dict(ft), root)
            cand_path = bundle.staging_dir / "candidate.json"
            candidate = json.loads(cand_path.read_text(encoding="utf-8"))
            staging_refs = dict(candidate.get("staging_refs") or {})
            staging_refs["fidelity_truth_ref"] = ft_path.as_posix()
            candidate["staging_refs"] = staging_refs
            _write_json(cand_path, candidate, root)
            bundle.staging_refs = staging_refs

    if tactical_buffer is not None and config.capture_normalization_enabled is False:
        append_tactical_capture_audits(
            audit,
            export_audit,
            session_id=session.session_id,
            capture_candidate_id=bundle.capture_candidate_id,
            command_id=command_id,
            issued_by=issued_by,
            annex=tactical_annex,
            tactical_annex_ref=tactical_annex_ref,
            buffer=tactical_buffer,
        )

    norm_result = None
    if config.capture_normalization_enabled:
        norm_ctx = build_normalization_context_from_session(
            session, config, telemetry_subs
        )
        norm_ctx.tactical_annex = tactical_annex
        norm_ctx.tactical_annex_ref = tactical_annex_ref
        try:
            norm_result = normalize_capture_bundle(
                bundle.staging_dir,
                runtime_context=norm_ctx,
                repo_root=root,
                config=config,
            )
            total_size = staging_bundle_size(bundle.staging_dir)
            if total_size > config.max_capture_bundle_bytes:
                mark_normalization_rejected(
                    bundle.staging_dir,
                    reason="bundle exceeds size cap after normalization",
                    repo_root=root,
                )
                raise NormalizationError(
                    "RESOURCE_LIMIT_EXCEEDED",
                    "capture bundle exceeds size cap after normalization",
                )
            export_audit.append(
                "provenance_injected",
                capture_candidate_id=bundle.capture_candidate_id,
                session_id=session.session_id,
                result="OK",
            )
            export_audit.append(
                "capture_normalized",
                capture_candidate_id=bundle.capture_candidate_id,
                session_id=session.session_id,
                result="OK",
                detail={
                    "conversion_revision": norm_result.conversion_revision,
                    "input_content_hash": norm_result.input_content_hash,
                },
            )
            val_errors = validate_normalized_capture(bundle.staging_dir)
            export_audit.append(
                "normalization_validation",
                capture_candidate_id=bundle.capture_candidate_id,
                session_id=session.session_id,
                result="OK" if not val_errors else "FAIL",
                detail={"errors": val_errors} if val_errors else None,
            )
            if val_errors:
                mark_normalization_rejected(
                    bundle.staging_dir,
                    reason="; ".join(val_errors),
                    repo_root=root,
                )
                raise NormalizationError("INVALID_STATE", "; ".join(val_errors))
            if tactical_buffer is not None:
                append_tactical_capture_audits(
                    audit,
                    export_audit,
                    session_id=session.session_id,
                    capture_candidate_id=bundle.capture_candidate_id,
                    command_id=command_id,
                    issued_by=issued_by,
                    annex=tactical_annex,
                    tactical_annex_ref=tactical_annex_ref,
                    buffer=tactical_buffer,
                )
            if norm_result.pose_assessment is not None:
                append_capture_pose_audits(
                    audit,
                    session.session_id,
                    norm_result.pose_assessment,
                    capture_candidate_id=bundle.capture_candidate_id,
                    command_id=command_id,
                    issued_by=issued_by,
                )
                export_audit.append(
                    "export_pose_normalized",
                    capture_candidate_id=bundle.capture_candidate_id,
                    session_id=session.session_id,
                    result="OK",
                    detail=export_pose_normalized_detail(
                        norm_result.pose_assessment,
                        conversion_revision=norm_result.conversion_revision,
                        input_content_hash=norm_result.input_content_hash,
                    ),
                )
            if norm_result.fidelity_assessment is not None:
                append_fidelity_capture_audits(
                    audit,
                    session.session_id,
                    norm_result.fidelity_assessment,
                    capture_candidate_id=bundle.capture_candidate_id,
                    command_id=command_id,
                    issued_by=issued_by,
                )
                export_audit.append(
                    "fidelity_capture_snapshot",
                    capture_candidate_id=bundle.capture_candidate_id,
                    session_id=session.session_id,
                    result="OK",
                    detail=export_fidelity_capture_detail(
                        norm_result.fidelity_assessment,
                        conversion_revision=norm_result.conversion_revision,
                        input_content_hash=norm_result.input_content_hash,
                    ),
                )
        except NormalizationError as exc:
            mark_normalization_rejected(
                bundle.staging_dir,
                reason=exc.message,
                repo_root=root,
            )
            export_audit.append(
                "normalization_rejected",
                capture_candidate_id=bundle.capture_candidate_id,
                session_id=session.session_id,
                result=exc.code,
                detail={"message": exc.message},
            )
            audit.append(
                session.session_id,
                command_id=command_id,
                command_type="capture_session",
                issued_by=issued_by,
                result=exc.code,
                detail={"message": exc.message},
            )
            return fail(base, exc.code, exc.message)

    session.cleanup_after = None
    session.state = SessionState.CAPTURED
    teardown_capture_post_session(
        session,
        TeardownDeps(audit=audit, telemetry_subs=telemetry_subs),
        command_id=command_id,
        issued_by=issued_by,
    )

    detail = {
        "state": session.state.value,
        "capture_candidate_id": bundle.capture_candidate_id,
        "staging_refs": bundle.staging_refs,
    }
    if workflow_summary:
        detail["workflow_summary"] = workflow_summary
    if templates_applied:
        detail["templates_applied"] = templates_applied
    audit.append(
        session.session_id,
        command_id=command_id,
        command_type="capture_session",
        issued_by=issued_by,
        result="OK",
        detail=detail,
    )
    export_audit.append(
        "capture_validated",
        capture_candidate_id=bundle.capture_candidate_id,
        session_id=session.session_id,
        result="OK",
        detail={"staging_refs": bundle.staging_refs},
    )

    resp = ok(base, state=session.state.value)
    resp["governance_banner"] = CAPTURE_GOVERNANCE_BANNER
    resp["capture_candidate_id"] = bundle.capture_candidate_id
    resp["staging_refs"] = bundle.staging_refs
    if norm_result is not None:
        resp["normalization_status"] = "normalized"
        resp["normalized_staging_refs"] = {
            k: v
            for k, v in norm_result.staging_refs.items()
            if k
            in {
                "normalized_manifest_ref",
                "provenance_ref",
                "validation_ref",
            }
        }
        resp["conversion_revision"] = norm_result.conversion_revision
    elif not config.capture_normalization_enabled:
        resp["normalization_status"] = "pending"
    return resp
