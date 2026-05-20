#!/usr/bin/env python3
"""Shared helpers for PLAT-SA-STAB platform integrity audits."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

_REPO = Path(__file__).resolve().parents[2]
FIXTURES_SA = _REPO / "fixtures/sa_r0"
PUBLIC_DEMO = _REPO / "platform/sa-r0-viewer/public/demo"
SCENARIOS = _REPO / "fixtures/scenarios"

SYNTHESIS_PARITY_FILES = (
    "cross_sweep_synthesis_v1.json",
    "replay_linkage_index_v1.json",
    "storyline_linkage_overlay_v1.json",
    "cross_sweep_summary.md",
    "linkage_summary.md",
    "cognition_rollup_summary.md",
    "cross_sweep_publication_report.md",
)

SWEEP_REPORT_PARITY_SUFFIXES = (
    "analytics_summary.md",
    "topology_sweep_summary.md",
    "sweep_narrative_summary.md",
    "replay_cluster_report.md",
    "topology_divergence_report.md",
    "ambiguity_hotspot_summary.md",
    "replay_compare_report_v1.json",
    "replay_review_report_v1.json",
    "publication_packet.html",
    "review_packet.html",
    "sweep_presentation_packet.md",
    "guided_walkthrough_report.md",
    "replay_presentation_report_v1.json",
)


def repo_root() -> Path:
    return _REPO


def assert_json_files_equal(path_a: Path, path_b: Path, label: str) -> str | None:
    if not path_a.is_file():
        return f"{label}: missing fixture {path_a}"
    if not path_b.is_file():
        return f"{label}: missing public mirror {path_b}"
    a = json.loads(path_a.read_text(encoding="utf-8"))
    b = json.loads(path_b.read_text(encoding="utf-8"))
    if a != b:
        return f"{label} out of sync: {path_a} vs {path_b}"
    return None


def assert_text_files_equal(path_a: Path, path_b: Path, label: str) -> str | None:
    if not path_a.is_file():
        return f"{label}: missing fixture {path_a}"
    if not path_b.is_file():
        return f"{label}: missing public mirror {path_b}"
    if path_a.read_text(encoding="utf-8") != path_b.read_text(encoding="utf-8"):
        return f"{label} out of sync: {path_a} vs {path_b}"
    return None


def assert_file_bytes_equal(path_a: Path, path_b: Path, label: str) -> str | None:
    if not path_a.is_file():
        return f"{label}: missing fixture {path_a}"
    if not path_b.is_file():
        return f"{label}: missing public mirror {path_b}"
    if path_a.read_bytes() != path_b.read_bytes():
        return f"{label} bytes differ: {path_a} vs {path_b}"
    return None


def parity_file(
    fixture_root: Path,
    pub_root: Path,
    rel: str,
    *,
    json_file: bool = False,
) -> str | None:
    a = fixture_root / rel
    b = pub_root / rel
    if json_file or rel.endswith(".json"):
        return assert_json_files_equal(a, b, rel)
    if rel.endswith(".md") or rel.endswith(".html"):
        return assert_text_files_equal(a, b, rel)
    return assert_file_bytes_equal(a, b, rel)


def collect_sweep_ids() -> tuple[str, ...]:
    from build_cross_sweep_synthesis import SWEEP_IDS  # noqa: WPS433

    return SWEEP_IDS


def iter_presentation_json_pairs() -> Iterable[tuple[Path, Path]]:
    fix = FIXTURES_SA / "presentations"
    pub = PUBLIC_DEMO / "presentations"
    if not fix.is_dir():
        return
    for path in sorted(fix.glob("*.json")):
        yield path, pub / path.name


def catalog_pack_ids() -> set[str]:
    catalog = json.loads((SCENARIOS / "index.json").read_text(encoding="utf-8"))
    return {str(p["pack_id"]) for p in catalog.get("packs") or [] if p.get("pack_id")}


def sweep_ids_from_index() -> set[str]:
    path = SCENARIOS / "sweeps_index_v1.json"
    if not path.is_file():
        return set()
    index = json.loads(path.read_text(encoding="utf-8"))
    return {str(s["sweep_id"]) for s in index.get("sweeps") or [] if s.get("sweep_id")}


def storyboard_ids() -> set[str]:
    path = FIXTURES_SA / "presentations/index.json"
    if not path.is_file():
        return set()
    index = json.loads(path.read_text(encoding="utf-8"))
    return {str(s["storyboard_id"]) for s in index.get("storyboards") or [] if s.get("storyboard_id")}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))
