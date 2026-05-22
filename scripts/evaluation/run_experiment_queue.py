#!/usr/bin/env python3
"""Run offline experiment job queue from manifest (PLAT-SA-H3)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from experiment_orchestration import run_manifest  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="Run experiment job manifest (offline, sequential)")
    ap.add_argument("--manifest", type=Path, required=True, help="Path to experiment_job_manifest_v1 JSON")
    ap.add_argument("--dry-run", action="store_true", help="Plan steps without executing subprocesses")
    ap.add_argument("--job", help="Run single job_id from manifest")
    ap.add_argument("--allow-runtime-capture", action="store_true", help="Allow runtime_capture steps")
    ap.add_argument(
        "--allow-async-worker",
        action="store_true",
        help="Allow async worker provenance metadata in report (PLAT-SA-I2; never default)",
    )
    ap.add_argument("--write-queue", action="store_true", default=True, help="Write queue snapshot JSON")
    ap.add_argument("--no-write-queue", action="store_false", dest="write_queue")
    ap.add_argument("--queue-id", help="Override queue snapshot id")
    ap.add_argument("--json", action="store_true", help="Print report JSON to stdout")
    args = ap.parse_args()

    result = run_manifest(
        args.manifest.resolve(),
        dry_run=args.dry_run,
        job_filter=args.job,
        allow_runtime_capture=args.allow_runtime_capture,
        allow_async_worker=args.allow_async_worker,
        write_queue=args.write_queue,
    )
    failed = [j for j in result["queue"]["jobs"] if j.get("status") == "failed"]
    if args.json:
        print(json.dumps(result["report"], indent=2))
    else:
        for job in result["queue"]["jobs"]:
            print(f"{job['job_id']}: {job['status']} phase={job.get('phase')}")
        if failed:
            print(f"FAILED: {len(failed)} job(s)", file=sys.stderr)
    raise SystemExit(1 if failed and not args.dry_run else 0)


if __name__ == "__main__":
    main()
