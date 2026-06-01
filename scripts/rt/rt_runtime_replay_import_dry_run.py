#!/usr/bin/env python3
"""Read-only dry-run preview for RT runtime_run.json → replay_sa_bundle_v1 mapping (D1 Step 2)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from rt_runtime_replay_bundle import (  # noqa: E402
    build_dry_run_preview,
    load_runtime_run,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Preview RT runtime_run.json → replay_sa_bundle_v1 mapping. "
            "Read-only — no writes, no corpus commit."
        ),
    )
    parser.add_argument(
        "runtime_run_json",
        type=Path,
        help="Path to runtime_run.json (rt_runtime_run_capture_v1)",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON preview on stdout")
    args = parser.parse_args()

    path = args.runtime_run_json
    if not path.is_file():
        print(f"file not found: {path}", file=sys.stderr)
        return 1

    artifact = load_runtime_run(path)
    resolved = path.resolve()
    try:
        source_ref = str(resolved.relative_to(_REPO))
    except ValueError:
        source_ref = str(resolved)
    preview = build_dry_run_preview(artifact, source_path=source_ref)

    if args.json:
        print(json.dumps(preview, indent=2, sort_keys=True))
    else:
        print(f"schema: {preview.get('schema')}")
        print(f"mapping_ok: {preview.get('mapping_ok')}")
        print(f"tracks_count: {preview.get('tracks_count')}")
        print(f"entities_count: {preview.get('entities_count')}")
        print(f"markers: {len(preview.get('markers') or [])}")
        if preview.get("missing_fields_warnings"):
            print("warnings:")
            for warning in preview["missing_fields_warnings"]:
                print(f"  - {warning}")
    return 0 if preview.get("mapping_ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
