#!/usr/bin/env python3
"""Maintainer-only capture approval and conversion manifest (no SA import)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.capture import (  # noqa: E402
    CaptureBundleError,
    write_approval_record,
    write_conversion_manifest,
)
from rt_sandbox.export_audit_log import ExportAuditLog  # noqa: E402
from rt_sandbox.export_boundary import validate_conversion_manifest  # noqa: E402
from rt_sandbox.isolation import repo_root_from, rt_sandbox_captures_dir  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Approve RT capture and write conversion manifest (no SA import)"
    )
    parser.add_argument("capture_id", help="capture_candidate_id / staging dir name")
    parser.add_argument("--approved-by", default="maintainer_cli", help="approver id")
    parser.add_argument(
        "--scenario-pack-ref",
        default=None,
        help="optional fixtures/scenarios/... path for conversion manifest",
    )
    parser.add_argument(
        "--log-path",
        default=None,
        help="optional log path reference for conversion manifest (not imported)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate only; do not write approval or conversion files",
    )
    parser.add_argument(
        "--skip-normalization-check",
        action="store_true",
        help="allow approval without normalization (legacy test fixtures only)",
    )
    args = parser.parse_args()

    repo_root = repo_root_from()
    staging_dir = rt_sandbox_captures_dir(repo_root) / args.capture_id
    if not staging_dir.is_dir():
        print(f"Staging dir not found: {staging_dir}", file=sys.stderr)
        return 1

    if args.dry_run:
        cand = json.loads((staging_dir / "candidate.json").read_text(encoding="utf-8"))
        print(json.dumps({"candidate": cand, "dry_run": True}, indent=2))
        return 0

    try:
        approval = write_approval_record(
            staging_dir,
            approved_by=args.approved_by,
            skip_normalization_check=args.skip_normalization_check,
        )
        manifest = write_conversion_manifest(
            staging_dir,
            scenario_pack_ref=args.scenario_pack_ref,
            log_path=args.log_path,
        )
        err = validate_conversion_manifest(manifest)
        if err:
            print(f"Conversion manifest invalid: {err}", file=sys.stderr)
            return 1
        ExportAuditLog(repo_root).append(
            "capture_approved",
            capture_candidate_id=args.capture_id,
            result="OK",
            detail={"approved_by": args.approved_by},
        )
        ExportAuditLog(repo_root).append(
            "conversion_manifest_written",
            capture_candidate_id=args.capture_id,
            result="OK",
        )
        print(json.dumps({"approval": approval, "conversion": manifest}, indent=2))
        return 0
    except CaptureBundleError as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        ExportAuditLog(repo_root).append(
            "capture_rejected",
            capture_candidate_id=args.capture_id,
            result=exc.code,
            detail={"message": exc.message},
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
