#!/usr/bin/env python3
"""Export deterministic replay corpus release archive zip (PLAT-SA-F1d)."""

from __future__ import annotations

import argparse
import json
import sys
import zipfile
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_corpus_lineage import sha256_file  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_RELEASE = "sa_r0_corpus_r1_r1"
_ARCHIVE_NAME = "sa_r0_corpus_release_archive.zip"


def _archive_members(release_id: str) -> list[tuple[Path, str]]:
    """Return (source_path, arcname) pairs for archive."""
    pairs: list[tuple[Path, str]] = []
    release_dir = _REPO / "fixtures/sa_r0/corpus_releases" / release_id
    for path in sorted(release_dir.rglob("*")):
        if path.is_file() and path.name != _ARCHIVE_NAME:
            arc = path.relative_to(release_dir).as_posix()
            pairs.append((path, arc))

    extras = [
        _REPO / "fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json",
        _REPO / "fixtures/sa_r0/synthesis/replay_corpus_evolution_summary_v1.json",
        _REPO / "fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json",
        _REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_drift_report_v1.json",
        _REPO / "fixtures/sa_r0/corpus_audits/replay_corpus_release_diff_v1.json",
    ]
    for path in extras:
        if path.is_file():
            pairs.append((path, f"extras/{path.name}"))

    return sorted(pairs, key=lambda x: x[1])


def build_archive(release_id: str = _DEFAULT_RELEASE) -> Path:
    release_dir = _REPO / "fixtures/sa_r0/corpus_releases" / release_id
    archive_dir = release_dir / "archive"
    archive_dir.mkdir(parents=True, exist_ok=True)
    out = archive_dir / _ARCHIVE_NAME

    manifest_rows: list[dict[str, Any]] = []
    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for src, arc in _archive_members(release_id):
            zf.write(src, arcname=arc)
            digest = sha256_file(src)
            manifest_rows.append(
                {
                    "arcname": arc,
                    "source_path": src.relative_to(_REPO).as_posix(),
                    "sha256": digest,
                    "size_bytes": src.stat().st_size,
                }
            )

    sidecar = {
        "release_id": release_id,
        "archive_name": _ARCHIVE_NAME,
        "archive_sha256": sha256_file(out),
        "members": manifest_rows,
    }
    (archive_dir / "archive_manifest.json").write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return out


def check_archive(release_id: str = _DEFAULT_RELEASE) -> None:
    expected_path = (
        _REPO / "fixtures/sa_r0/corpus_releases" / release_id / "archive" / _ARCHIVE_NAME
    )
    sidecar_path = expected_path.parent / "archive_manifest.json"
    if not expected_path.is_file() or not sidecar_path.is_file():
        raise SystemExit("missing release archive — run export_replay_corpus_release.py")

    rebuilt = build_archive(release_id)
    if sha256_file(rebuilt) != sha256_file(expected_path):
        raise SystemExit("release archive zip is stale — run export_replay_corpus_release.py")


def main() -> None:
    ap = argparse.ArgumentParser(description="Export replay corpus release archive")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--release", default=_DEFAULT_RELEASE)
    args = ap.parse_args()

    if args.check:
        check_archive(args.release)
        print("corpus release export check OK")
        return

    path = build_archive(args.release)
    print(f"corpus release export OK: {path}")


if __name__ == "__main__":
    main()
