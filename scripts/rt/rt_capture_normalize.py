#!/usr/bin/env python3
"""Maintainer-only capture re-normalization (no SA import)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.capture_normalize import (  # noqa: E402
    NormalizationContext,
    NormalizationError,
    mark_normalization_rejected,
    normalize_capture_bundle,
    validate_normalized_capture,
)
from rt_sandbox.capture_pose_cognition import export_pose_normalized_detail  # noqa: E402
from rt_sandbox.export_audit_log import ExportAuditLog  # noqa: E402
from rt_sandbox.governance import GovernanceConfig  # noqa: E402
from rt_sandbox.isolation import repo_root_from, rt_sandbox_captures_dir  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Re-normalize RT capture staging (no SA import)"
    )
    parser.add_argument("capture_id", help="capture_candidate_id / staging dir name")
    parser.add_argument(
        "--runtime-context-json",
        default=None,
        help="optional JSON file for NormalizationContext (tests/maintainer repair)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate raw artifacts only; do not write normalized files",
    )
    args = parser.parse_args()

    repo_root = repo_root_from()
    staging_dir = rt_sandbox_captures_dir(repo_root) / args.capture_id
    if not staging_dir.is_dir():
        print(f"Staging dir not found: {staging_dir}", file=sys.stderr)
        return 1

    if args.runtime_context_json:
        ctx_data = json.loads(Path(args.runtime_context_json).read_text(encoding="utf-8"))
        ctx = NormalizationContext(
            session_state=str(ctx_data.get("session_state", "stopped")),
            enable_gazebo_adapter=bool(ctx_data.get("enable_gazebo_adapter", False)),
            adapter_mode=str(ctx_data.get("adapter_mode", "mock")),
            adapter_attached=bool(ctx_data.get("adapter_attached", False)),
            enable_fidelity_coupling=bool(ctx_data.get("enable_fidelity_coupling", False)),
            pose_sync_summary=ctx_data.get("pose_sync_summary"),
            telemetry_mirror_summary=ctx_data.get("telemetry_mirror_summary"),
            telemetry_channel_history=list(ctx_data.get("telemetry_channel_history") or []),
            workflow_summary=ctx_data.get("workflow_summary"),
            templates_applied=ctx_data.get("templates_applied"),
            fidelity_truth_snapshot=ctx_data.get("fidelity_truth_snapshot"),
        )
    else:
        ctx = NormalizationContext(
            session_state="stopped",
            enable_gazebo_adapter=False,
            adapter_mode="stub",
            adapter_attached=False,
            enable_fidelity_coupling=False,
        )

    if args.dry_run:
        errors = validate_normalized_capture(staging_dir)
        print(
            json.dumps(
                {"capture_id": args.capture_id, "dry_run": True, "errors": errors},
                indent=2,
            )
        )
        return 1 if errors else 0

    config = GovernanceConfig()
    try:
        result = normalize_capture_bundle(
            staging_dir,
            runtime_context=ctx,
            repo_root=repo_root,
            config=config,
        )
        errors = validate_normalized_capture(staging_dir)
        audit = ExportAuditLog(repo_root)
        audit.append(
            "capture_normalized",
            capture_candidate_id=args.capture_id,
            result="OK",
            detail={"conversion_revision": result.conversion_revision, "re_normalize": True},
        )
        audit.append(
            "provenance_injected",
            capture_candidate_id=args.capture_id,
            result="OK",
        )
        audit.append(
            "normalization_validation",
            capture_candidate_id=args.capture_id,
            result="OK" if not errors else "FAIL",
            detail={"errors": errors} if errors else None,
        )
        if result.pose_assessment is not None and not errors:
            audit.append(
                "export_pose_normalized",
                capture_candidate_id=args.capture_id,
                result="OK",
                detail={
                    **export_pose_normalized_detail(
                        result.pose_assessment,
                        conversion_revision=result.conversion_revision,
                        input_content_hash=result.input_content_hash,
                    ),
                    "re_normalize": True,
                },
            )
        print(json.dumps({"normalization": result.__dict__, "validation_errors": errors}, indent=2, default=str))
        return 1 if errors else 0
    except NormalizationError as exc:
        mark_normalization_rejected(staging_dir, reason=exc.message, repo_root=repo_root)
        ExportAuditLog(repo_root).append(
            "normalization_rejected",
            capture_candidate_id=args.capture_id,
            result=exc.code,
            detail={"message": exc.message},
        )
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
