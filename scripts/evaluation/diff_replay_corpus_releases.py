#!/usr/bin/env python3
"""Diff corpus index snapshots (replay_corpus_release_diff_v1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_corpus_lineage import diff_corpus_indexes  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_AUDITS = _REPO / "fixtures/sa_r0/corpus_audits"
_DEFAULT_BASELINE = _REPO / "fixtures/sa_r0/corpus_releases/sa_r0_corpus_r1_r1/replay_corpus_index_v1.json"
_DEFAULT_TARGET = _REPO / "fixtures/sa_r0/synthesis/replay_corpus_index_v1.json"


def build_release_diff(
    baseline: dict[str, Any],
    target: dict[str, Any],
    *,
    baseline_id: str,
    target_id: str,
) -> dict[str, Any]:
    return diff_corpus_indexes(baseline, target, baseline_id=baseline_id, target_id=target_id)


def write_release_diff(diff: dict[str, Any]) -> None:
    _AUDITS.mkdir(parents=True, exist_ok=True)
    (_AUDITS / "replay_corpus_release_diff_v1.json").write_text(
        json.dumps(diff, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def check_release_diff(
    *,
    baseline_path: Path,
    target_path: Path,
    baseline_id: str,
    target_id: str,
    fail_on_diff: bool = False,
) -> None:
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    target = json.loads(target_path.read_text(encoding="utf-8"))
    expected = build_release_diff(baseline, target, baseline_id=baseline_id, target_id=target_id)
    path = _AUDITS / "replay_corpus_release_diff_v1.json"
    if not path.is_file():
        raise SystemExit(f"missing release diff: {path}")
    actual = json.loads(path.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit("replay_corpus_release_diff_v1.json is stale — run diff_replay_corpus_releases.py")
    if fail_on_diff and expected.get("release_behind_canonical"):
        raise SystemExit("canonical index differs from frozen release (run build_replay_corpus_release.py)")


def main() -> None:
    ap = argparse.ArgumentParser(description="Diff replay corpus index snapshots")
    ap.add_argument("--baseline", default=str(_DEFAULT_BASELINE))
    ap.add_argument("--target", default=str(_DEFAULT_TARGET))
    ap.add_argument("--baseline-id", default="sa_r0_corpus_r1_r1")
    ap.add_argument("--target-id", default="canonical_index")
    ap.add_argument("--check", action="store_true", help="verify committed diff fixture")
    ap.add_argument("--strict", action="store_true", help="fail if release behind canonical")
    ap.add_argument("--json", action="store_true", help="print diff JSON")
    args = ap.parse_args()

    baseline_path = Path(args.baseline)
    target_path = Path(args.target)
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    target = json.loads(target_path.read_text(encoding="utf-8"))
    diff = build_release_diff(
        baseline,
        target,
        baseline_id=args.baseline_id,
        target_id=args.target_id,
    )

    if args.check:
        check_release_diff(
            baseline_path=baseline_path,
            target_path=target_path,
            baseline_id=args.baseline_id,
            target_id=args.target_id,
            fail_on_diff=args.strict,
        )
        print("corpus release diff check OK")
        return

    if args.json:
        print(json.dumps(diff, indent=2, sort_keys=True))
        return

    write_release_diff(diff)
    behind = diff.get("release_behind_canonical")
    print(f"corpus release diff OK (release_behind_canonical={behind})")


if __name__ == "__main__":
    main()
