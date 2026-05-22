#!/usr/bin/env python3
"""Build deterministic replay federation index and related artifacts (PLAT-SA-F2A)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_federation_lineage import (  # noqa: E402
    build_federation_index,
    federation_fixture_paths,
    write_all_federation_artifacts,
)

_REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay federation artifacts")
    ap.add_argument(
        "--check",
        action="store_true",
        help="verify committed fixtures match rebuild",
    )
    ap.add_argument(
        "--write",
        action="store_true",
        help="write federation fixtures to disk",
    )
    args = ap.parse_args()

    expected = build_federation_index(_REPO)
    paths = federation_fixture_paths(_REPO)

    if args.write:
        write_all_federation_artifacts(_REPO)
        print(f"wrote federation artifacts under {paths['index'].parent}")
        return

    if not args.check:
        print(json.dumps(expected, indent=2, sort_keys=True))
        return

    index_path = paths["index"]
    if not index_path.is_file():
        raise SystemExit(f"missing committed fixture: {index_path}")

    actual = json.loads(index_path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit(
            "federation index stale — run: python3 scripts/evaluation/build_replay_federation_index.py --write"
        )
    print("build_replay_federation_index: OK")


if __name__ == "__main__":
    main()
