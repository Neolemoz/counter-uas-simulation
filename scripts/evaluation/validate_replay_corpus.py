#!/usr/bin/env python3
"""Validate replay corpus index (replay_corpus_index_v1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_corpus_lineage import (  # noqa: E402
    ENTRY_KINDS,
    EVOLUTION_TAG_VALUES,
    RELEASE_GENERATION_ID_VALUES,
    REVIEWER_CATEGORIES,
    validate_lineage_dag,
)

_REPO = Path(__file__).resolve().parents[2]
DEFAULT_INDEX = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"

_REQUIRED_ROOT = frozenset(
    {
        "artifact_type",
        "schema_version",
        "corpus_id",
        "generation_revision",
        "governance",
        "entries",
    }
)

_REQUIRED_ENTRY = frozenset(
    {
        "entry_id",
        "corpus_id",
        "entry_kind",
        "lineage_parent_ids",
        "derived_from",
        "source_artifacts",
        "generation_tool",
        "generation_revision",
        "primary_artifact_path",
        "sha256",
        "content_revision",
    }
)


def validate_replay_corpus_index(
    index: dict[str, Any],
    *,
    repo_root: Path | None = None,
    strict: bool = False,
) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    root = repo_root or _REPO

    for field in _REQUIRED_ROOT:
        if field not in index:
            errors.append(f"missing root field: {field}")

    if index.get("artifact_type") != "replay_corpus_index_v1":
        errors.append("artifact_type must be replay_corpus_index_v1")
    if index.get("schema_version") != "replay_corpus_index_v1":
        errors.append("schema_version must be replay_corpus_index_v1")

    gov = index.get("governance") or {}
    if not gov.get("notice"):
        warnings.append("governance.notice missing")

    entries = index.get("entries") or []
    if not entries:
        errors.append("entries must be non-empty")

    for i, entry in enumerate(entries):
        prefix = f"entries[{i}]"
        for field in _REQUIRED_ENTRY:
            if field not in entry:
                errors.append(f"{prefix} missing field: {field}")
        kind = entry.get("entry_kind")
        if kind and kind not in ENTRY_KINDS:
            errors.append(f"{prefix} unknown entry_kind: {kind}")
        cat = entry.get("reviewer_category")
        if cat and cat not in REVIEWER_CATEGORIES:
            warnings.append(f"{prefix} unknown reviewer_category: {cat}")
        if entry.get("navigation_tags") is not None and not isinstance(entry.get("navigation_tags"), list):
            errors.append(f"{prefix} navigation_tags must be a list")
        rgid = entry.get("release_generation_id")
        if rgid and rgid not in RELEASE_GENERATION_ID_VALUES:
            warnings.append(f"{prefix} unknown release_generation_id: {rgid}")
        for tag in entry.get("evolution_tags") or []:
            if tag not in EVOLUTION_TAG_VALUES:
                warnings.append(f"{prefix} unknown evolution_tag: {tag}")
        path_str = entry.get("primary_artifact_path")
        if path_str:
            full = root / path_str
            if not full.is_file():
                errors.append(f"{prefix} primary_artifact_path not found: {path_str}")
        sha = entry.get("sha256")
        if path_str and sha and (root / path_str).is_file():
            from replay_corpus_lineage import sha256_file  # noqa: E402

            actual = sha256_file(root / path_str)
            if actual != sha:
                errors.append(f"{prefix} sha256 mismatch for {path_str}")

    errors.extend(validate_lineage_dag(entries, index.get("lineage_edges")))

    if strict and warnings:
        errors.extend([f"strict: {w}" for w in warnings])

    return errors, warnings


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate replay corpus index")
    ap.add_argument(
        "index_path",
        nargs="?",
        default=str(DEFAULT_INDEX),
        help="Path to replay_corpus_index_v1.json",
    )
    ap.add_argument("--json", action="store_true", help="emit JSON result")
    ap.add_argument("--strict", action="store_true", help="treat warnings as errors")
    args = ap.parse_args()

    path = Path(args.index_path)
    if not path.is_file():
        raise SystemExit(f"missing index: {path}")

    index = json.loads(path.read_text(encoding="utf-8"))
    errors, warnings = validate_replay_corpus_index(index, strict=args.strict)

    if args.json:
        print(json.dumps({"ok": not errors, "errors": errors, "warnings": warnings}, indent=2))
    else:
        for w in warnings:
            print(f"warning: {w}")
        for e in errors:
            print(f"error: {e}")

    if errors:
        raise SystemExit(1)
    print("validate replay corpus OK")


if __name__ == "__main__":
    main()
