#!/usr/bin/env python3
"""Pack tactical annex sidecars from experiment manifest staging refs (PLAT-RT-F3).

Reads rt_experiment_manifest_v1, loads tactical_annex.json per capture_staging_ref,
writes rt_experiment_annex_bundle_v1 for RT workbench import. No bridge calls.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def _load_manifest(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != "rt_experiment_manifest_v1":
        raise ValueError("manifest must be rt_experiment_manifest_v1")
    return data


def _load_annex(staging_dir: Path) -> dict[str, Any] | None:
    annex_path = staging_dir / "tactical_annex.json"
    if not annex_path.is_file():
        return None
    data = json.loads(annex_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return None
    if data.get("schema") != "rt_tactical_capture_annex_v1":
        raise ValueError(f"unexpected annex schema in {annex_path}")
    if data.get("authority_label") != "replay_boundary_scoped":
        raise ValueError(f"annex authority must be replay_boundary_scoped: {annex_path}")
    return data


def pack_annexes(
    manifest: dict[str, Any],
    *,
    repo_root: Path,
) -> dict[str, Any]:
    entries: list[dict[str, Any]] = []
    for run in manifest.get("runs") or []:
        if not isinstance(run, dict):
            continue
        staging_ref = run.get("capture_staging_ref")
        if not isinstance(staging_ref, str) or not staging_ref:
            continue
        staging_dir = repo_root / staging_ref
        annex = _load_annex(staging_dir)
        if annex is None:
            continue
        entries.append(
            {
                "run_id": str(run["run_id"]),
                "capture_candidate_id": run.get("capture_candidate_id"),
                "annex": annex,
            }
        )
    return {
        "schema": "rt_experiment_annex_bundle_v1",
        "experiment_id": manifest.get("experiment_id"),
        "entries": entries,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Pack RT experiment annex bundle")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    parser.add_argument("--out", type=Path, help="write bundle JSON")
    args = parser.parse_args(argv)

    manifest = _load_manifest(args.manifest)
    bundle = pack_annexes(manifest, repo_root=args.repo_root.resolve())
    text = json.dumps(bundle, indent=2, sort_keys=True) + "\n"

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
