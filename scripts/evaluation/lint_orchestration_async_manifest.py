#!/usr/bin/env python3
"""Lint experiment_orchestration_async_manifest_v1 sidecars (PLAT-SA-I2)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_async as orch_async  # noqa: E402
import replay_sa_orchestration_ops as orch_ops  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Lint async orchestration sidecars")
    ap.add_argument("manifest_id", nargs="?", default="", help="Single manifest_id")
    ap.add_argument("--all-manifests", action="store_true", help="Lint all async sidecars")
    ap.add_argument("--check", action="store_true", help="Exit 1 on lint issues")
    ap.add_argument("--strict", action="store_true", help="Strict governance lint")
    args = ap.parse_args()

    ids: list[str] = []
    if args.all_manifests:
        ids = orch_async._list_async_manifest_ids()
        if not ids:
            print("lint_orchestration_async: no async sidecars (ok)")
            return 0
    elif args.manifest_id:
        ids = [args.manifest_id]
    else:
        ap.error("provide manifest_id or --all-manifests")

    failed = False
    for mid in ids:
        result = orch_async.lint_async_manifest(mid, strict=args.strict)
        for issue in result.get("issues") or []:
            print(f"ERROR {mid}: {issue}")
            failed = True
        for warn in result.get("warnings") or []:
            print(f"WARN {mid}: {warn}")

    if args.check and failed:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
