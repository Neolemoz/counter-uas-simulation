#!/usr/bin/env python3
"""RT→SA import pipeline dry-run preview only (PLAT-RT-F6 P2).

Never commits to corpus. Use rt_sa_import.py commit --corpus-dest for SA lineage.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
_SCRIPTS_RT = _REPO / "scripts" / "rt"
for p in (_BRIDGE_PKG, _SCRIPTS_RT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rt_sandbox.export_boundary import CONVERSION_STEPS  # noqa: E402
from rt_sandbox.isolation import repo_root_from  # noqa: E402
from rt_sandbox.sa_handoff import sa_handoff_dir  # noqa: E402

from rt_sa_import import cmd_prepare, cmd_run_step  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Preview RT→SA import prepare + pipeline with dry_run=True only. "
            "Never calls commit. SA lineage begins only at "
            "rt_sa_import commit --corpus-dest."
        ),
    )
    parser.add_argument("capture_id")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root or repo_root_from(Path.cwd())
    capture_id = args.capture_id

    log: dict[str, object] = {
        "schema": "rt_sa_import_dry_run_preview_v1",
        "capture_candidate_id": capture_id,
        "dry_run": True,
        "steps": [],
        "commit": "forbidden — use rt_sa_import commit --corpus-dest explicitly",
    }

    rc_prep = cmd_prepare(repo_root, capture_id, dry_run=True)
    log["steps"].append({"phase": "prepare", "rc": rc_prep, "dry_run": True})

    if rc_prep != 0:
        if args.json:
            print(json.dumps(log, indent=2, sort_keys=True))
        else:
            print("prepare dry-run failed", file=sys.stderr)
        return rc_prep

    for step in CONVERSION_STEPS:
        rc = cmd_run_step(repo_root, capture_id, step, dry_run=True)
        log["steps"].append({"phase": step, "rc": rc, "dry_run": True})
        if rc != 0:
            break
    else:
        rc = 0

    handoff_path = sa_handoff_dir(repo_root, capture_id)
    preview_path = handoff_path / "dry_run_preview.json"
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    preview_path.write_text(json.dumps(log, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json:
        log["preview_path"] = str(preview_path.relative_to(repo_root))
        print(json.dumps(log, indent=2, sort_keys=True))
    else:
        print(f"dry-run preview written: {preview_path}")
        print("commit: forbidden in this CLI — use rt_sa_import commit --corpus-dest")

    return rc


if __name__ == "__main__":
    raise SystemExit(main())
