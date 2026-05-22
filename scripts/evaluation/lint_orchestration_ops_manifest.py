#!/usr/bin/env python3
"""Lint experiment_orchestration_ops_manifest_v1 sidecars (PLAT-SA-I1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_ops as orch_ops  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Lint orchestration ops sidecars")
    ap.add_argument("paths", nargs="*", type=Path, help="Ops JSON or manifest file (optional)")
    ap.add_argument("--all-manifests", action="store_true", help="Lint ops for every job manifest")
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--check", action="store_true", help="Exit 1 on issues")
    args = ap.parse_args()

    targets: list[str] = []
    if args.all_manifests:
        for mf in orch_ops.list_all_manifest_files():
            targets.append(str(load_manifest(mf)["manifest_id"]))
    for p in args.paths:
        p = p.resolve()
        if p.name.endswith("_ops.json"):
            targets.append(p.stem.removesuffix("_ops"))
        elif p.suffix == ".json":
            targets.append(str(load_manifest(p)["manifest_id"]))

    all_issues: list[str] = []
    for mid in sorted(set(targets)):
        result = orch_ops.lint_ops_manifest(mid, strict=args.strict)
        for issue in result.get("issues") or []:
            all_issues.append(issue)
        for warn in result.get("warnings") or []:
            if args.strict:
                all_issues.append(warn)
            else:
                print(f"WARN: {warn}")

    if all_issues:
        for issue in all_issues:
            print(issue, file=sys.stderr)
        if args.check:
            return 1
    else:
        print(f"lint_orchestration_ops_manifest OK ({len(targets)} manifest(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
