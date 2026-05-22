#!/usr/bin/env python3
"""Lint experiment_job_manifest_v1 files (PLAT-SA-H3)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from experiment_orchestration import lint_manifest, load_manifest  # noqa: E402
from governance_lint_sa import lint_markdown_text  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]


def _lint_file(path: Path) -> list[str]:
    issues: list[str] = []
    try:
        data = load_manifest(path)
    except (json.JSONDecodeError, ValueError) as exc:
        return [f"{path}: {exc}"]
    issues.extend(lint_manifest(data, path=str(path)))
    text = path.read_text(encoding="utf-8")
    issues.extend(lint_markdown_text(text, context=str(path)))
    gov = data.get("governance") or {}
    for field in ("notice",):
        val = str(gov.get(field, ""))
        issues.extend(lint_markdown_text(val, context=f"{path}: governance.{field}"))
    return issues


def main() -> None:
    ap = argparse.ArgumentParser(description="Lint experiment manifests")
    ap.add_argument("paths", nargs="+", type=Path, help="Manifest file or directory")
    ap.add_argument("--check", action="store_true", help="Exit 1 if any issues")
    args = ap.parse_args()

    targets: list[Path] = []
    for p in args.paths:
        if p.is_dir():
            targets.extend(sorted(p.glob("*.json")))
        else:
            targets.append(p)

    all_issues: list[str] = []
    for path in targets:
        all_issues.extend(_lint_file(path.resolve()))

    if all_issues:
        for issue in all_issues:
            print(issue, file=sys.stderr)
        if args.check:
            raise SystemExit(1)
    else:
        print(f"lint_experiment_manifest OK ({len(targets)} file(s))")


if __name__ == "__main__":
    main()
