#!/usr/bin/env python3
"""Lint scenario_authoring_manifest_v1 sidecars (PLAT-SA-A1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_authoring as authoring  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Lint authoring_manifest.json for a scenario pack.")
    parser.add_argument("pack_dir", type=Path, nargs="?", help="Pack directory")
    parser.add_argument(
        "--all-experiment-packs",
        action="store_true",
        help="Lint valley experiment variant packs (alias: subset of catalog)",
    )
    parser.add_argument(
        "--all-catalog-packs",
        action="store_true",
        help="Lint all packs listed in fixtures/scenarios/index.json",
    )
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2] / "fixtures" / "scenarios"
    packs: list[Path] = []
    if args.all_catalog_packs:
        import replay_sa_authoring_integrity as integrity  # noqa: E402

        for pack_id in integrity.catalog_pack_ids():
            packs.append(root / pack_id)
    elif args.all_experiment_packs:
        for name in (
            "valley_ingress",
            "valley_ingress_radar_shifted_north",
            "valley_ingress_extra_valley_sensor",
            "valley_ingress_reduced_overlap_layout",
            "valley_ingress_delayed_interceptor_base",
        ):
            packs.append(root / name)
    elif args.pack_dir:
        packs = [args.pack_dir]
    else:
        parser.error("provide pack_dir, --all-catalog-packs, or --all-experiment-packs")

    exit_code = 0
    results: dict[str, dict] = {}
    for pack in packs:
        result = authoring.lint_authoring_manifest(pack, strict=args.strict)
        results[pack.name] = result
        if not result.get("ok"):
            exit_code = 1

    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        for name, result in results.items():
            status = "OK" if result.get("ok") else "FAIL"
            print(f"{name}: {status}")
            for issue in result.get("issues") or []:
                print(f"  ERROR: {issue}")
            for warn in result.get("warnings") or []:
                print(f"  WARN: {warn}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
