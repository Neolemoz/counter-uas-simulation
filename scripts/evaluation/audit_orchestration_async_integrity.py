#!/usr/bin/env python3
"""Corpus-wide async orchestration integrity audit CLI (PLAT-SA-I2)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_async as orch_async  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Async orchestration corpus integrity audit")
    ap.add_argument("--strict", action="store_true", help="Treat async issues as errors")
    ap.add_argument("--json", action="store_true", help="Emit full report JSON")
    args = ap.parse_args()

    result = orch_async.run_async_integrity_audit(strict=args.strict)

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        status = "PASS" if result.get("ok") else "FAIL"
        issue_count = len(result.get("issues") or [])
        warn_count = len(result.get("warnings") or [])
        print(
            f"async_integrity: {status} "
            f"({len(result.get('per_manifest') or {})} async manifests, "
            f"{issue_count} issues, {warn_count} warnings)"
        )
        for issue in result.get("issues") or []:
            print(f"ERROR [{issue.get('kind')}]: {issue.get('manifest_id')}: {issue.get('message')}")

    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
