#!/usr/bin/env python3
"""Validate scenario_topology_v1 packs (deterministic, agent-friendly CLI)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL_DIR = Path(__file__).resolve().parent
if str(_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(_EVAL_DIR))

from replay_sa_scenario import lint_scenario_pack  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate scenario_topology_v1 pack directory.")
    parser.add_argument("pack_dir", type=Path, help="Path to fixtures/scenarios/<id>/")
    parser.add_argument("--json", action="store_true", help="Emit JSON result only")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as errors")
    args = parser.parse_args()

    result = lint_scenario_pack(args.pack_dir, strict=args.strict)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        for issue in result.get("issues") or []:
            print(f"ERROR: {issue}", file=sys.stderr)
        for warn in result.get("warnings") or []:
            print(f"WARN: {warn}", file=sys.stderr)
        if result.get("ok"):
            print(f"OK: {args.pack_dir}")
        else:
            print(f"FAILED: {args.pack_dir}", file=sys.stderr)

    if not result.get("ok"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
