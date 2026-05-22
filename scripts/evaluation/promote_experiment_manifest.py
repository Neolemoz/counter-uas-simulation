#!/usr/bin/env python3
"""Promote experiment_job_manifest_v1 through orchestration ops workflow (PLAT-SA-I1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_ops as orch_ops  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Manage experiment_orchestration_ops_manifest_v1 (CLI authority only).",
    )
    parser.add_argument("manifest_file", type=Path, help="Path to experiment job manifest JSON")
    parser.add_argument("--status", choices=orch_ops.OPERATIONS_STATUSES, help="Target operations_status")
    parser.add_argument("--init", action="store_true", help="Create pending ops sidecar")
    parser.add_argument("--record-validation", action="store_true", help="Lint manifest + verify mirrors")
    parser.add_argument("--record-queue", action="store_true", help="Record queue snapshot refs (dry-run queue)")
    parser.add_argument("--check-stale", action="store_true", help="Report stale manifest fingerprint")
    parser.add_argument("--lint-ops", action="store_true", help="Lint ops sidecar only")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without writing")
    parser.add_argument("--allow-stale", action="store_true", help="Allow promote when fingerprint stale")
    parser.add_argument("--notes", default="", help="Notes stored on promote")
    parser.add_argument("--json", action="store_true", help="Emit JSON result")
    parser.add_argument("--strict", action="store_true", help="Strict ops lint")
    parser.add_argument("--summary", action="store_true", help="Include operations summary in output")
    parser.add_argument("--repro-check", action="store_true", help="Verify lint + mirrors + fingerprint")
    parser.add_argument("--lineage-report", action="store_true", help="Execution lineage report JSON")
    parser.add_argument("--replay-continuity", action="store_true", help="Replay continuity summary JSON")
    parser.add_argument("--queue-id", default="", help="Override default queue id for --record-queue")
    args = parser.parse_args()

    manifest_file = args.manifest_file.resolve()

    if args.lint_ops:
        mid = str(orch_ops.load_manifest(manifest_file)["manifest_id"])
        result = orch_ops.lint_ops_manifest(mid, strict=args.strict)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            for issue in result.get("issues") or []:
                print(f"ERROR: {issue}")
            for warn in result.get("warnings") or []:
                print(f"WARN: {warn}")
        return 0 if result.get("ok") else 1

    if args.check_stale:
        result = orch_ops.check_stale(manifest_file)
        print(json.dumps(result, indent=2))
        return 0

    if args.init:
        result = orch_ops.init_ops_for_manifest(manifest_file, dry_run=args.dry_run)
        print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.record_validation:
        result = orch_ops.record_validation(manifest_file, dry_run=args.dry_run)
        print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.record_queue:
        result = orch_ops.record_queue(
            manifest_file,
            queue_id=args.queue_id or None,
            dry_run=True,
        )
        print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.repro_check:
        result = orch_ops.repro_check(manifest_file)
        print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.lineage_report:
        result = orch_ops.execution_lineage_report(manifest_file)
        print(json.dumps(result, indent=2))
        return 0

    if args.replay_continuity:
        result = orch_ops.replay_continuity_summary(manifest_file)
        print(json.dumps(result, indent=2))
        return 0

    if not args.status:
        parser.error(
            "provide --status, --init, --record-validation, --record-queue, "
            "--check-stale, --lint-ops, --repro-check, --lineage-report, or --replay-continuity"
        )

    result = orch_ops.promote_experiment_manifest(
        manifest_file,
        target_status=args.status,
        notes=args.notes,
        dry_run=args.dry_run,
        allow_stale=args.allow_stale,
        record_validation_first=args.status in ("validated", "queued", "executed", "replay_generated"),
        record_queue_dry_run=args.status in ("queued", "executed"),
    )
    if args.summary and result.get("ok") and not result.get("dry_run"):
        mid = str(orch_ops.load_manifest(manifest_file)["manifest_id"])
        sp = orch_ops._OPS / f"{mid}_{orch_ops.OPS_SUMMARY_FILENAME}"
        if sp.is_file():
            result["operations_summary"] = json.loads(sp.read_text(encoding="utf-8"))

    if args.json or args.summary:
        print(json.dumps(result, indent=2))
    elif result.get("ok"):
        print(
            f"promote: {result.get('manifest_id')} "
            f"{result.get('from_status', '?')} -> {result.get('operations_status')}"
        )
    else:
        print(f"promote failed: {result.get('error')}", file=sys.stderr)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
