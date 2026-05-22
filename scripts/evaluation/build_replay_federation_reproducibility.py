#!/usr/bin/env python3
"""Build replay federation reproducibility fingerprint (PLAT-SA-F2A)."""

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
    write_federation_json,
)

_REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    ap = argparse.ArgumentParser(description="Build federation reproducibility artifact")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    index = build_federation_index(_REPO)
    expected = build_federation_reproducibility(_REPO, index)
    paths = federation_fixture_paths(_REPO)

    if args.write:
        write_federation_json(paths["reproducibility"], expected)
        print(f"wrote {paths['reproducibility']}")
        return

    if args.check:
        path = paths["reproducibility"]
        if not path.is_file():
            raise SystemExit(f"missing: {path}")
        actual = json.loads(path.read_text(encoding="utf-8"))
        if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
            raise SystemExit("federation reproducibility stale — run with --write")
        print("build_replay_federation_reproducibility: OK")
        return

    print(json.dumps(expected, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
