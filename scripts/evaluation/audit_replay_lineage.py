#!/usr/bin/env python3
"""Audit replay corpus lineage: orphans, stale entries, duplicates."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_replay_corpus_index import build_corpus_index  # noqa: E402
from replay_corpus_lineage import collect_drift_findings, validate_lineage_dag  # noqa: E402
from validate_replay_corpus import validate_replay_corpus_index  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
DEFAULT_INDEX = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"


def audit_replay_lineage(
    index: dict[str, Any],
    *,
    repo_root: Path | None = None,
) -> list[str]:
    root = repo_root or _REPO
    issues: list[str] = []

    val_errors, _ = validate_replay_corpus_index(index, repo_root=root)
    issues.extend(val_errors)
    issues.extend(validate_lineage_dag(index.get("entries") or [], index.get("lineage_edges")))

    try:
        expected_index = build_corpus_index()
    except Exception as exc:
        expected_index = None
        issues.append(f"rebuild comparison failed: {exc}")

    for finding in collect_drift_findings(index, repo_root=root, expected_index=expected_index):
        severity = finding.get("severity", "info")
        if severity == "error":
            issues.append(f"{finding.get('kind')}: {finding.get('message')}")
        elif severity == "warning" and finding.get("kind") in (
            "stale_sha256",
            "index_stale",
            "viewer_mirror_stale",
            "corpus_ref_mismatch",
        ):
            issues.append(f"{finding.get('kind')}: {finding.get('message')}")

    return issues


def main() -> None:
    ap = argparse.ArgumentParser(description="Audit replay corpus lineage")
    ap.add_argument(
        "index_path",
        nargs="?",
        default=str(DEFAULT_INDEX),
        help="Path to replay_corpus_index_v1.json",
    )
    ap.add_argument("--json", action="store_true", help="emit JSON result")
    args = ap.parse_args()

    path = Path(args.index_path)
    if not path.is_file():
        raise SystemExit(f"missing index: {path}")

    index = json.loads(path.read_text(encoding="utf-8"))
    issues = audit_replay_lineage(index)

    if args.json:
        print(json.dumps({"ok": not issues, "issues": issues}, indent=2))
    else:
        for issue in issues:
            print(issue)

    if issues:
        raise SystemExit(1)
    print("audit replay lineage OK")


if __name__ == "__main__":
    main()
