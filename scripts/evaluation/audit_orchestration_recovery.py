#!/usr/bin/env python3
"""Async recovery and reconciliation integrity audit (PLAT-SA-I3)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_recovery as recovery  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit async recovery/reconciliation (explanatory, CLI authority).",
    )
    parser.add_argument("--strict", action="store_true", help="Treat recovery gaps as errors")
    parser.add_argument("--json", action="store_true", help="Emit JSON report")
    parser.add_argument("--manifest-id", default="", help="Audit single manifest only")
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Refresh recovery fixtures and write async_batch_audit_v1.json",
    )
    parser.add_argument(
        "--refresh-fixtures",
        action="store_true",
        help="Write recovery reports and lineage index to fixtures/",
    )
    args = parser.parse_args()

    if args.refresh_fixtures or args.batch:
        result = recovery.refresh_recovery_artifacts(strict=args.strict)
        report = result["recovery"]
    elif args.manifest_id:
        mid = args.manifest_id
        issues = recovery.audit_recovery_continuity(mid, strict=args.strict)
        report = {
            "ok": len(issues) == 0,
            "manifest_id": mid,
            "issues": issues,
            "retry_chain": recovery.build_retry_chain(mid),
            "recovery_report": recovery.build_recovery_report(mid),
        }
    else:
        report = recovery.run_recovery_integrity_audit(strict=args.strict)

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        ok = report.get("ok", True)
        issues = report.get("issues") or []
        print(f"recovery integrity: ok={ok} issues={len(issues)}")
        for item in issues:
            print(f"ERROR: [{item.get('kind')}] {item.get('manifest_id')}: {item.get('message')}")
        if args.batch:
            print(f"batch audit refreshed: {report.get('batch_path', 'fixtures/.../async_batch_audit_v1.json')}")

    return 0 if report.get("ok", True) and not report.get("error") else 1


if __name__ == "__main__":
    raise SystemExit(main())
