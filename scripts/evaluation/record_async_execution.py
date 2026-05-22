#!/usr/bin/env python3
"""Record async orchestration bookkeeping (PLAT-SA-I2, CLI authority only)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_async as orch_async  # noqa: E402
import replay_sa_orchestration_recovery as orch_recovery  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Manage async orchestration artifacts (CLI authority only).",
    )
    parser.add_argument("manifest_file", type=Path, help="Path to experiment job manifest JSON")
    parser.add_argument("--init-async", action="store_true", help="Create async sidecar")
    parser.add_argument("--claim", action="store_true", help="Record queue claim token")
    parser.add_argument("--record-worker", action="store_true", help="Record worker execution provenance")
    parser.add_argument(
        "--set-status",
        choices=sorted(orch_async.ASYNC_EXECUTION_STATUSES),
        help="Set async_execution_status",
    )
    parser.add_argument("--execution-fingerprint", action="store_true", help="Compute and store fingerprint")
    parser.add_argument("--continuity-report", action="store_true", help="Queue execution continuity JSON")
    parser.add_argument("--recovery-report", action="store_true", help="Write recovery report JSON (read-only)")
    parser.add_argument(
        "--continuity-snapshot",
        action="store_true",
        help="Alias: recovery continuity snapshot via recovery report",
    )
    parser.add_argument("--summary", action="store_true", help="Write async execution summary")
    parser.add_argument("--worker-id", default="cli-worker-local", help="Worker id for claim/record")
    parser.add_argument("--execution-attempt", type=int, default=1, help="Worker attempt number")
    parser.add_argument("--queue-id", default="", help="Override default queue id")
    parser.add_argument("--retry-parent-ref", default="", help="Prior worker record ref")
    parser.add_argument("--notes", default="", help="Notes for --set-status lineage")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without writing")
    parser.add_argument("--json", action="store_true", help="Emit JSON result")
    parser.add_argument("--allow-runtime-capture", action="store_true", help="Record in deterministic_metadata")
    args = parser.parse_args()

    manifest_file = args.manifest_file.resolve()
    if not manifest_file.is_file():
        print(f"manifest not found: {manifest_file}", file=sys.stderr)
        return 1

    result: dict[str, object] = {}

    if args.init_async:
        result = orch_async.init_async_for_manifest(manifest_file, dry_run=args.dry_run)
    elif args.claim:
        result = orch_async.build_claim_token(
            manifest_file,
            worker_id=args.worker_id,
            queue_id=args.queue_id or None,
            dry_run=args.dry_run,
        )
    elif args.record_worker:
        result = orch_async.build_worker_record(
            manifest_file,
            worker_id=args.worker_id,
            execution_attempt=args.execution_attempt,
            retry_parent_ref=args.retry_parent_ref or None,
            dry_run=args.dry_run,
            allow_runtime_capture=args.allow_runtime_capture,
            allow_async_worker=True,
        )
    elif args.set_status:
        mid = str(load_manifest(manifest_file)["manifest_id"])
        result = orch_async.set_async_status(
            mid, args.set_status, notes=args.notes, dry_run=args.dry_run
        )
    elif args.execution_fingerprint:
        result = orch_async.store_execution_fingerprint(manifest_file, dry_run=args.dry_run)
    elif args.continuity_report:
        result = orch_async.queue_execution_continuity(manifest_file)
    elif args.recovery_report or args.continuity_snapshot:
        mid = str(load_manifest(manifest_file)["manifest_id"])
        path = orch_recovery.write_recovery_report(mid)
        result = {
            "ok": True,
            "manifest_id": mid,
            "recovery_report_path": str(path.relative_to(orch_recovery._REPO)),
            "report": orch_recovery.build_recovery_report(mid),
        }
    elif args.summary:
        path = orch_async.write_async_execution_summary(manifest_file)
        result = {"ok": path is not None, "summary_path": str(path) if path else None}
    else:
        parser.error(
            "provide --init-async, --claim, --record-worker, --set-status, "
            "--execution-fingerprint, --continuity-report, --recovery-report, or --summary"
        )

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(json.dumps(result, indent=2))

    return 0 if result.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
