#!/usr/bin/env python3
"""CLI for corpus-wide scenario authoring integrity (PLAT-SA-A2)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_authoring_integrity as integrity  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit scenario authoring integrity across catalog.")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as errors where applicable")
    parser.add_argument("--json", action="store_true", help="Emit JSON report")
    parser.add_argument(
        "--lineage-report",
        action="store_true",
        help="Emit lineage continuity report for one pack",
    )
    parser.add_argument("--pack-id", type=str, help="Pack id for --lineage-report")
    args = parser.parse_args()

    if args.lineage_report:
        if not args.pack_id:
            parser.error("--lineage-report requires --pack-id")
        report = integrity.lineage_continuity_report(args.pack_id)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    result = integrity.run_integrity_audit(strict=args.strict)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        status = "OK" if result.get("ok") else "FAIL"
        print(f"authoring_integrity: {status} ({result.get('manifest_count')}/{result.get('pack_count')} manifests)")
        for err in result.get("errors") or []:
            print(f"ERROR: {err}")
        for warn in result.get("warnings") or []:
            print(f"WARN: {warn}")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
