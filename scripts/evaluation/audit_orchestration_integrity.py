#!/usr/bin/env python3
"""Corpus-wide orchestration integrity audit CLI (PLAT-SA-I1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_integrity as integrity  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Orchestration corpus integrity audit")
    ap.add_argument("--strict", action="store_true", help="Treat warnings as errors where applicable")
    ap.add_argument("--json", action="store_true", help="Emit full report JSON")
    ap.add_argument("--batch", action="store_true", help="Batch summary (default with no other flags)")
    ap.add_argument("--pipeline", choices=("validation-only", "full"), help="Filter manifest audit set")
    ap.add_argument("--repro-strict", action="store_true", help="Strict reproducibility fingerprint checks")
    ap.add_argument("--lineage-report", action="store_true")
    ap.add_argument("--manifest-id", default="", help="With --lineage-report")
    args = ap.parse_args()

    if args.lineage_report:
        if not args.manifest_id:
            print("--manifest-id required with --lineage-report", file=sys.stderr)
            return 1
        report = integrity.lineage_continuity_report(args.manifest_id)
        print(json.dumps(report, indent=2))
        return 0 if report.get("ok", True) and not report.get("error") else 1

    result = integrity.run_integrity_audit(
        strict=args.strict,
        pipeline_filter=args.pipeline,
        repro_strict=args.repro_strict,
    )

    if args.batch:
        synth_dir = Path(__file__).resolve().parents[2] / "fixtures" / "orchestration" / "synthesis"
        synth_dir.mkdir(parents=True, exist_ok=True)
        out = synth_dir / "orchestration_batch_audit_v1.json"
        batch = {
            "artifact_type": "orchestration_batch_audit_v1",
            "schema_version": "1",
            "generated_at": result.get("checked_at"),
            "ok": result.get("ok"),
            "manifest_count": result.get("manifest_count"),
            "pack_count": result.get("pack_count"),
            "error_count": len(result.get("errors") or []),
            "warning_count": len(result.get("warnings") or []),
            "governance_banner": "BATCH AUDIT — explanatory export only",
        }
        out.write_text(json.dumps(batch, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(f"orchestration integrity: ok={result.get('ok')} errors={len(result.get('errors') or [])} "
              f"warnings={len(result.get('warnings') or [])}")
        for err in result.get("errors") or []:
            print(f"ERROR: {err}")
        for warn in result.get("warnings") or []:
            print(f"WARN: {warn}")

    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
