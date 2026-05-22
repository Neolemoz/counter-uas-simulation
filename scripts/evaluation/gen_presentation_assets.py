#!/usr/bin/env python3
"""Deterministic presentation asset generation (thumbnails and summary cards)."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import SWEEP_IDS, _load_sweep  # noqa: E402
from classify_replay_pattern import PATTERN_LABELS, primary_pattern  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
FIG_DPI = 100
CARD_W, CARD_H = 4.0, 2.5

PATTERN_COLORS = {
    "los_fragmented_replay": "#4C78A8",
    "assignment_instability_replay": "#F58518",
    "delayed_detection_replay": "#E45756",
    "corridor_pressure_replay": "#72B7B2",
    "saturation_driven_ambiguity": "#54A24B",
    "topology_sensitive_divergence": "#B279A2",
}


def _card(fig, ax, title: str, subtitle: str, color: str = "#f4f4f5") -> None:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, facecolor=color, edgecolor="#999", linewidth=1))
    ax.text(0.05, 0.65, title, fontsize=11, fontweight="bold", va="top", wrap=True)
    ax.text(0.05, 0.25, subtitle, fontsize=8, va="top", color="#333", wrap=True)
    ax.text(0.05, 0.05, "Explanatory replay card — not scoring", fontsize=6, color="#666")


def render_topology_summary_card(manifest: dict[str, Any], out_path: Path) -> None:
    sid = manifest["sweep_id"]
    baseline = manifest.get("baseline_topology_key", "")
    count = len(manifest.get("members") or [])
    fig, ax = plt.subplots(figsize=(CARD_W, CARD_H))
    _card(
        fig,
        ax,
        f"Topology summary — {sid}",
        f"Baseline: {baseline}\nMembers: {count}",
        "#eef2ff",
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def render_sweep_cohort_thumbnail(manifest: dict[str, Any], out_path: Path) -> None:
    sid = manifest["sweep_id"]
    members = manifest.get("members") or []
    tags = [primary_pattern(m.get("replay_pattern_tags") or []) or "other" for m in members]
    fig, ax = plt.subplots(figsize=(CARD_W, CARD_H))
    subtitle = "\n".join(f"• {t}" for t in tags[:4]) or "no patterns"
    _card(fig, ax, f"Cohort — {sid}", subtitle, "#fef3c7")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def render_pattern_cards(out_dir: Path) -> None:
    for pid, label in PATTERN_LABELS.items():
        fig, ax = plt.subplots(figsize=(CARD_W, CARD_H))
        color = PATTERN_COLORS.get(pid, "#e5e7eb")
        _card(fig, ax, pid.replace("_", " ").title(), label, color)
        out_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_dir / f"pattern_{pid}.png", dpi=FIG_DPI, bbox_inches="tight")
        plt.close(fig)


def render_chapter_thumbnails(manifest: dict[str, Any], out_dir: Path) -> None:
    sid = manifest["sweep_id"]
    walk = manifest.get("presentation_walkthrough") or {}
    out_dir.mkdir(parents=True, exist_ok=True)
    for i, step in enumerate(walk.get("steps") or []):
        fig, ax = plt.subplots(figsize=(CARD_W, CARD_H))
        _card(
            fig,
            ax,
            f"Ch.{i + 1}: {step.get('label', 'Step')}",
            str(step.get("copy", ""))[:120],
            "#ecfdf5",
        )
        fig.savefig(out_dir / f"{sid}_chapter_{i + 1}.png", dpi=FIG_DPI, bbox_inches="tight")
        plt.close(fig)


def export_all_assets() -> dict[str, Any]:
    fixture_assets = _REPO / "fixtures/sa_r0/synthesis/assets"
    pub_assets = _REPO / "platform/sa-r0-viewer/public/demo/synthesis/assets"
    index_entries: list[dict[str, Any]] = []

    for d in (fixture_assets, pub_assets):
        render_pattern_cards(d / "patterns")

    for sid in SWEEP_IDS:
        manifest = _load_sweep(sid)
        for d in (fixture_assets, pub_assets):
            render_topology_summary_card(manifest, d / f"{sid}_topology_card.png")
            render_sweep_cohort_thumbnail(manifest, d / f"{sid}_cohort_thumb.png")
            render_chapter_thumbnails(manifest, d / "chapters")
        index_entries.append({"sweep_id": sid, "topology_card": f"{sid}_topology_card.png"})

    index = {
        "artifact_type": "presentation_assets_index_v1",
        "schema_version": "presentation_assets_index_v1",
        "sweeps": index_entries,
        "pattern_cards": list(PATTERN_LABELS.keys()),
    }
    for d in (fixture_assets, pub_assets):
        (d / "index.json").write_text(json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return index


def main() -> None:
    export_all_assets()
    print("presentation assets OK")


if __name__ == "__main__":
    main()
