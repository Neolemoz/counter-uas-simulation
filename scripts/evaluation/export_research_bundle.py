#!/usr/bin/env python3
"""Export portable replay research bundles (replay_research_bundle_v1)."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import zipfile
from pathlib import Path
from typing import Any

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import SWEEP_IDS  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
CORPUS_ID = "sa_r0_corpus_r1"
GENERATOR_VERSIONS = {
    "build_cross_sweep_synthesis.py": "e2_v1",
    "build_replay_linkage.py": "e2_v1",
    "build_replay_cognition_rollup.py": "e2_v1",
    "build_replay_corpus_index.py": "f1a_v1",
    "export_research_bundle.py": "f1a_v1",
}


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _collect_files(bundle_dir: Path) -> list[dict[str, Any]]:
    files: list[dict[str, Any]] = []
    for path in sorted(bundle_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            rel = path.relative_to(bundle_dir).as_posix()
            files.append(
                {
                    "path": rel,
                    "sha256": _sha256(path),
                    "size_bytes": path.stat().st_size,
                }
            )
    return files


def build_research_bundle(corpus_id: str = CORPUS_ID) -> Path:
    out = _REPO / "fixtures/sa_r0/research_bundles" / corpus_id
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    synth_src = _REPO / "fixtures/sa_r0/synthesis"
    synth_dst = out / "synthesis"
    if synth_src.is_dir():
        shutil.copytree(synth_src, synth_dst, dirs_exist_ok=True)
    corpus_index = synth_src / "replay_corpus_index_v1.json"
    if corpus_index.is_file():
        shutil.copy2(corpus_index, synth_dst / "replay_corpus_index_v1.json")

    fig_src = synth_src / "figures"
    if fig_src.is_dir():
        shutil.copytree(fig_src, out / "figures", dirs_exist_ok=True)

    assets_src = synth_src / "assets"
    if assets_src.is_dir():
        shutil.copytree(assets_src, out / "assets", dirs_exist_ok=True)

    for sweep_id in SWEEP_IDS:
        reports_src = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "reports"
        if reports_src.is_dir():
            dst = out / "sweeps" / sweep_id / "reports"
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(reports_src, dst, dirs_exist_ok=True)

    prov = out / "provenance"
    prov.mkdir(parents=True, exist_ok=True)
    (prov / "generator_versions.json").write_text(
        json.dumps(GENERATOR_VERSIONS, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    included = _collect_files(out)
    from replay_corpus_lineage import attach_corpus_ref  # noqa: E402

    manifest = {
        "artifact_type": "replay_research_bundle_v1",
        "schema_version": "replay_research_bundle_v1",
        "corpus_id": corpus_id,
        "governance": {
            "notice": (
                "Portable offline replay research bundle — explanatory artifacts only, "
                "not operational deployment or validated doctrine."
            ),
        },
        "sweep_ids": list(SWEEP_IDS),
        "included_files": included,
        "provenance": {
            "generators": GENERATOR_VERSIONS,
            "source_root": "fixtures/sa_r0",
        },
    }
    manifest = attach_corpus_ref(manifest, "research_bundle", corpus_id)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return out


def export_zip(bundle_dir: Path, zip_path: Path) -> None:
    zip_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(bundle_dir.rglob("*")):
            if path.is_file():
                zf.write(path, path.relative_to(bundle_dir).as_posix())


def check_research_bundle(corpus_id: str = CORPUS_ID) -> None:
    bundle_dir = _REPO / "fixtures/sa_r0/research_bundles" / corpus_id
    manifest_path = bundle_dir / "manifest.json"
    if not manifest_path.is_file():
        raise SystemExit(f"missing research bundle: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for entry in manifest.get("included_files") or []:
        path = bundle_dir / entry["path"]
        if not path.is_file():
            raise SystemExit(f"missing bundled file: {entry['path']}")
        if _sha256(path) != entry["sha256"]:
            raise SystemExit(f"sha256 mismatch: {entry['path']}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Export replay research bundle")
    ap.add_argument("--corpus", default=CORPUS_ID)
    ap.add_argument("--zip", action="store_true", help="also write zip archive")
    ap.add_argument("--check", action="store_true", help="verify bundle integrity")
    args = ap.parse_args()

    if args.check:
        check_research_bundle(args.corpus)
        print("research bundle check OK")
        return

    bundle_dir = build_research_bundle(args.corpus)
    if args.zip:
        zip_path = _REPO / "fixtures/sa_r0/research_bundles" / f"{args.corpus}.zip"
        export_zip(bundle_dir, zip_path)
        print(f"wrote {zip_path}")
    print(f"research bundle OK: {bundle_dir}")


if __name__ == "__main__":
    main()
