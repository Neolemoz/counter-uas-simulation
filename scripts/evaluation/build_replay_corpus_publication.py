#!/usr/bin/env python3
"""Build replay corpus publication packet (replay_corpus_publication_packet_v1)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import SWEEP_IDS  # noqa: E402
from replay_corpus_lineage import (  # noqa: E402
    CORPUS_ID,
    PUBLICATION_GENERATION_REVISION,
    PUBLICATION_GOVERNANCE,
    publication_revision_fingerprint,
    sha256_file,
    write_viewer_mirror,
)

_REPO = Path(__file__).resolve().parents[2]
_SA = _REPO / "fixtures/sa_r0"
_OUT = _SA / "synthesis/replay_corpus_publication_packet_v1.json"
_DEFAULT_RELEASE = f"{CORPUS_ID}_r1"


def _artifact_row(path: Path, kind: str) -> dict[str, Any]:
    rel = path.relative_to(_REPO).as_posix()
    return {
        "path": rel,
        "sha256": sha256_file(path),
        "kind": kind,
    }


def build_publication_packet(release_id: str = _DEFAULT_RELEASE) -> dict[str, Any]:
    included: list[dict[str, Any]] = []

    for sweep_id in SWEEP_IDS:
        pub = _SA / "sweeps" / sweep_id / "reports/replay_publication_report_v1.json"
        if pub.is_file():
            included.append(_artifact_row(pub, "publication_packet"))

    for name, kind in (
        ("replay_corpus_evolution_manifest_v1.json", "evolution_manifest"),
        ("replay_corpus_evolution_summary_v1.json", "evolution_summary"),
        ("replay_corpus_index_v1.json", "corpus_index"),
        ("cross_sweep_synthesis_v1.json", "synthesis"),
        ("replay_linkage_index_v1.json", "linkage"),
    ):
        path = _SA / "synthesis" / name
        if path.is_file():
            included.append(_artifact_row(path, kind))

    rb = _SA / "research_bundles" / CORPUS_ID / "manifest.json"
    if rb.is_file():
        included.append(_artifact_row(rb, "research_bundle"))

    rel_manifest = _SA / "corpus_releases" / release_id / "replay_corpus_release_manifest_v1.json"
    if rel_manifest.is_file():
        included.append(_artifact_row(rel_manifest, "release_manifest"))

    drift = _SA / "corpus_audits/replay_corpus_drift_report_v1.json"
    if drift.is_file():
        included.append(_artifact_row(drift, "drift_report"))

    rel_diff = _SA / "corpus_audits/replay_corpus_release_diff_v1.json"
    if rel_diff.is_file():
        included.append(_artifact_row(rel_diff, "release_diff"))

    included = sorted(included, key=lambda x: x["path"])
    paths = [a["path"] for a in included]

    return {
        "artifact_type": "replay_corpus_publication_packet_v1",
        "schema_version": "replay_corpus_publication_packet_v1",
        "corpus_id": CORPUS_ID,
        "release_id": release_id,
        "generation_revision": PUBLICATION_GENERATION_REVISION,
        "governance": PUBLICATION_GOVERNANCE,
        "included_artifacts": included,
        "publication_revision": publication_revision_fingerprint(paths),
    }


def write_publication_packet(packet: dict[str, Any]) -> None:
    _OUT.parent.mkdir(parents=True, exist_ok=True)
    _OUT.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    mirror = _REPO / "platform/sa-r0-viewer/public/demo/synthesis/replay_corpus_publication_packet_v1.json"
    write_viewer_mirror(_OUT, mirror)


def check_publication_packet() -> None:
    expected = build_publication_packet()
    if not _OUT.is_file():
        raise SystemExit("missing publication packet — run build_replay_corpus_publication.py")
    actual = json.loads(_OUT.read_text(encoding="utf-8"))
    if json.dumps(actual, sort_keys=True) != json.dumps(expected, sort_keys=True):
        raise SystemExit("replay_corpus_publication_packet_v1.json is stale")
    for row in expected["included_artifacts"]:
        full = _REPO / row["path"]
        if not full.is_file():
            raise SystemExit(f"publication artifact missing: {row['path']}")
        if sha256_file(full) != row["sha256"]:
            raise SystemExit(f"publication sha256 mismatch: {row['path']}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay corpus publication packet")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--release", default=_DEFAULT_RELEASE)
    args = ap.parse_args()

    if args.check:
        check_publication_packet()
        print("corpus publication packet check OK")
        return

    write_publication_packet(build_publication_packet(args.release))
    print("corpus publication packet OK")


if __name__ == "__main__":
    main()
