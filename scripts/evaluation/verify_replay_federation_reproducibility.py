#!/usr/bin/env python3
"""Verify committed federation reproducibility matches rebuild (PLAT-SA-F2A)."""

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
    build_federation_reproducibility,
    federation_fixture_paths,
)

_REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(description="Verify federation reproducibility")
    ap.add_argument("--write-diff", action="store_true")
    args = ap.parse_args()

    paths = federation_fixture_paths(_REPO)
    path = paths["reproducibility"]
    if not path.is_file():
        raise SystemExit(f"missing: {path}")

    index = build_federation_index(_REPO)
    expected = build_federation_reproducibility(_REPO, index)
    actual = json.loads(path.read_text(encoding="utf-8"))

    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        if args.write_diff:
            diff_path = path.parent / "reproducibility_diff.json"
            diff_path.write_text(
                json.dumps({"expected": expected, "actual": actual}, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"wrote diff to {diff_path}")
        raise SystemExit("federation reproducibility mismatch")
    print("verify_replay_federation_reproducibility: OK")


if __name__ == "__main__":
    main()
