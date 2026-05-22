#!/usr/bin/env python3
"""Promote scenario_topology_v1 packs through authoring workflow (PLAT-SA-A1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_authoring as authoring  # noqa: E402
import replay_sa_scenario as scenario_mod  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Manage scenario_authoring_manifest_v1 promotion (CLI authority only).",
    )
    parser.add_argument("pack_dir", type=Path, help="Path to fixtures/scenarios/<pack_id>")
    parser.add_argument(
        "--status",
        choices=authoring.PROMOTION_STATUSES,
        help="Target promotion_status",
    )
    parser.add_argument("--init", action="store_true", help="Create draft authoring_manifest.json")
    parser.add_argument(
        "--record-validation",
        action="store_true",
        help="Run lint, write validation mirror, set status validated",
    )
    parser.add_argument("--check-stale", action="store_true", help="Report stale validation fingerprint")
    parser.add_argument("--lint-manifest", action="store_true", help="Lint authoring_manifest.json only")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without writing")
    parser.add_argument("--allow-stale", action="store_true", help="Allow promote when fingerprint stale")
    parser.add_argument("--notes", default="", help="Authoring notes stored on promote")
    parser.add_argument("--json", action="store_true", help="Emit JSON result")
    parser.add_argument("--strict", action="store_true", help="Strict manifest lint")
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Include promotion summary in JSON output (with --json)",
    )
    parser.add_argument(
        "--repro-check",
        action="store_true",
        help="Verify lint + validation mirror + fingerprint",
    )
    parser.add_argument(
        "--diff-since-last-promote",
        action="store_true",
        help="Report changes since last promotion_lineage event",
    )
    args = parser.parse_args()

    pack_dir = args.pack_dir.resolve()

    if args.lint_manifest:
        result = authoring.lint_authoring_manifest(pack_dir, strict=args.strict)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            for issue in result.get("issues") or []:
                print(f"ERROR: {issue}")
            for warn in result.get("warnings") or []:
                print(f"WARN: {warn}")
        return 0 if result.get("ok") else 1

    if args.check_stale:
        result = authoring.check_stale(pack_dir)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(json.dumps(result, indent=2))
        return 0

    if args.init:
        if authoring.manifest_path(pack_dir).is_file() and not args.dry_run:
            print("manifest already exists", file=sys.stderr)
            return 1
        manifest = authoring.build_manifest_draft(pack_dir)
        lint = scenario_mod.lint_scenario_pack(pack_dir)
        if lint.get("ok") and manifest.get("promotion_status") == "draft":
            manifest["promotion_status"] = "linted"
        if args.dry_run:
            print(json.dumps({"ok": True, "dry_run": True, "manifest": manifest}, indent=2))
            return 0
        authoring._write_json(authoring.manifest_path(pack_dir), manifest)
        print(f"init: {authoring.manifest_path(pack_dir)}")
        return 0

    if args.record_validation:
        result = authoring.record_validation(pack_dir, dry_run=args.dry_run)
        if args.json:
            print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.repro_check:
        result = authoring.repro_check(pack_dir)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if args.diff_since_last_promote:
        result = authoring.diff_since_last_promote(pack_dir)
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 1

    if not args.status:
        parser.error(
            "provide --status, --init, --record-validation, --check-stale, "
            "--lint-manifest, --repro-check, or --diff-since-last-promote"
        )

    result = authoring.promote_scenario_pack(
        pack_dir,
        target_status=args.status,
        notes=args.notes,
        dry_run=args.dry_run,
        allow_stale=args.allow_stale,
        record_validation_first=args.status in ("validated", "promoted", "orchestration_ready"),
    )
    if args.summary and result.get("ok") and not result.get("dry_run"):
        summary_path = pack_dir / authoring.PROMOTION_SUMMARY_FILENAME
        if summary_path.is_file():
            result["promotion_summary"] = json.loads(summary_path.read_text(encoding="utf-8"))

    if args.json or args.summary:
        print(json.dumps(result, indent=2))
    elif result.get("ok"):
        print(
            f"promote: {result.get('pack_id')} "
            f"{result.get('from_status', '?')} -> {result.get('promotion_status')}"
        )
    else:
        print(f"promote failed: {result.get('error')}", file=sys.stderr)
        if result.get("lint"):
            print(json.dumps(result["lint"], indent=2), file=sys.stderr)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
