#!/usr/bin/env python3
"""Build deterministic replay corpus release snapshot (replay_corpus_release_manifest_v1)."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_replay_corpus_index import build_corpus_index, write_corpus_index  # noqa: E402
from replay_corpus_lineage import (  # noqa: E402
    CORPUS_ID,
    GOVERNANCE,
    discover_corpus_releases,
    publication_revision_fingerprint,
    sha256_file,
    write_viewer_mirror,
)

_REPO = Path(__file__).resolve().parents[2]
RELEASE_ID = f"{CORPUS_ID}_r1"
GENERATORS = {
    "build_replay_corpus_index.py": "f1a_v1",
    "build_replay_corpus_release.py": "f1a_v1",
}


def _collect_files(snapshot_dir: Path) -> list[dict[str, Any]]:
    files: list[dict[str, Any]] = []
    for path in sorted(snapshot_dir.rglob("*")):
        if path.is_file() and path.name != "replay_corpus_release_manifest_v1.json":
            rel = path.relative_to(snapshot_dir).as_posix()
            files.append(
                {
                    "path": rel,
                    "sha256": sha256_file(path),
                    "size_bytes": path.stat().st_size,
                }
            )
    return files


def build_release_snapshot(
    release_id: str = RELEASE_ID,
    *,
    parent_release_ids: list[str] | None = None,
) -> Path:
    index = build_corpus_index()
    out = _REPO / "fixtures/sa_r0/corpus_releases" / release_id
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    index_text = json.dumps(index, indent=2, sort_keys=True) + "\n"
    (out / "replay_corpus_index_v1.json").write_text(index_text, encoding="utf-8")

    entry_ids = [e["entry_id"] for e in index.get("entries") or []]
    files = _collect_files(out)
    parents = list(parent_release_ids or [])
    if not parents:
        for rel in discover_corpus_releases(_REPO):
            rid = rel["release_id"]
            if rid != release_id:
                parents.append(rid)
        parents = sorted(set(parents))

    evolution_chain: list[dict[str, Any]] = []
    for rel in discover_corpus_releases(_REPO):
        idx = rel.get("index") or {}
        evolution_chain.append(
            {
                "release_id": rel["release_id"],
                "parent_release_ids": (rel.get("manifest") or {}).get("parent_release_ids") or [],
                "index_revision": idx.get("index_revision"),
            }
        )
    evolution_chain.append(
        {
            "release_id": "canonical_index",
            "parent_release_ids": [r["release_id"] for r in discover_corpus_releases(_REPO)],
            "index_revision": index.get("index_revision"),
        }
    )

    pub_paths = [
        f"fixtures/sa_r0/corpus_releases/{release_id}/replay_corpus_index_v1.json",
        f"fixtures/sa_r0/corpus_releases/{release_id}/replay_corpus_release_manifest_v1.json",
    ]
    manifest = {
        "artifact_type": "replay_corpus_release_manifest_v1",
        "schema_version": "replay_corpus_release_manifest_v1",
        "release_id": release_id,
        "corpus_id": CORPUS_ID,
        "parent_release_ids": parents,
        "snapshot_root": f"fixtures/sa_r0/corpus_releases/{release_id}",
        "indexed_entry_ids": entry_ids,
        "governance": GOVERNANCE,
        "provenance": {
            "generators": GENERATORS,
            "source_root": "fixtures/sa_r0",
        },
        "files": files,
        "evolution_chain": evolution_chain,
        "publication_revision": publication_revision_fingerprint(pub_paths),
    }
    (out / "replay_corpus_release_manifest_v1.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    aggregate = sha256_file(out / "replay_corpus_release_manifest_v1.json")
    (out / "MANIFEST.sha256").write_text(f"{aggregate}\n", encoding="utf-8")

    manifest_path = out / "replay_corpus_release_manifest_v1.json"
    mirror_dir = _REPO / "platform/sa-r0-viewer/public/demo/corpus_releases" / release_id
    write_viewer_mirror(manifest_path, mirror_dir / "replay_corpus_release_manifest_v1.json")

    return out


def check_release(release_id: str = RELEASE_ID) -> None:
    out = _REPO / "fixtures/sa_r0/corpus_releases" / release_id
    manifest_path = out / "replay_corpus_release_manifest_v1.json"
    if not manifest_path.is_file():
        raise SystemExit(f"missing release manifest: {manifest_path}")

    expected_index = build_corpus_index()
    actual_index_path = out / "replay_corpus_index_v1.json"
    actual_index = json.loads(actual_index_path.read_text(encoding="utf-8"))
    if json.dumps(actual_index, sort_keys=True) != json.dumps(expected_index, sort_keys=True):
        raise SystemExit("release replay_corpus_index_v1.json stale — run build_replay_corpus_release.py")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for entry in manifest.get("files") or []:
        path = out / entry["path"]
        if not path.is_file():
            raise SystemExit(f"missing release file: {entry['path']}")
        if sha256_file(path) != entry["sha256"]:
            raise SystemExit(f"sha256 mismatch: {entry['path']}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Build replay corpus release snapshot")
    ap.add_argument("--check", action="store_true", help="verify committed release")
    ap.add_argument("--release", default=RELEASE_ID)
    ap.add_argument(
        "--refresh-index",
        action="store_true",
        help="also rewrite canonical corpus index before snapshot",
    )
    ap.add_argument(
        "--parent-release",
        action="append",
        default=[],
        help="parent release id(s) for evolution chain",
    )
    args = ap.parse_args()

    if args.check:
        check_release(args.release)
        print("corpus release check OK")
        return

    if args.refresh_index:
        write_corpus_index(build_corpus_index())

    out = build_release_snapshot(
        args.release,
        parent_release_ids=args.parent_release or None,
    )
    print(f"corpus release OK: {out}")


if __name__ == "__main__":
    main()
