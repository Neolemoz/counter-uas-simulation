#!/usr/bin/env python3
"""Static sweep-level figures for E2 publication (EVAL-VIZ-E2 slice)."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from build_cross_sweep_synthesis import SWEEP_IDS, _load_sweep  # noqa: E402
from classify_replay_pattern import PATTERN_LABELS, primary_pattern  # noqa: E402

matplotlib.rcParams["svg.hashsalt"] = "static_viz_e2_sweep_v1"
RENDER_PROFILE = "static_viz_e2_sweep_v1"
FIG_DPI = 120
_REPO = Path(__file__).resolve().parents[2]

LAYER_TITLES = {
    "ambiguity_density": "Ambiguity hotspot density",
    "los_degraded": "LOS degradation concentration",
    "topology_sensitivity": "Topology sensitivity grid",
}


def _grid_shape(grid: dict[str, Any]) -> tuple[int, int]:
    size = grid.get("size") or [40, 30]
    return int(size[0]), int(size[1])


def _counts_to_grid(counts: list[int], grid: dict[str, Any]) -> np.ndarray:
    cols, rows = _grid_shape(grid)
    arr = np.array(counts, dtype=float).reshape(rows, cols)
    return arr


def render_layer_heatmap(
    manifest: dict[str, Any],
    layer_key: str,
    out_path: Path,
) -> dict[str, Any]:
    sid = manifest["sweep_id"]
    spatial = manifest.get("spatial_aggregate") or {}
    grid = spatial.get("grid") or {}
    layer = (spatial.get("layers") or {}).get(layer_key) or {}
    counts = layer.get("counts") or []
    if not counts:
        return {"skipped": True, "reason": "empty_layer", "path": str(out_path)}

    data = _counts_to_grid(counts, grid)
    fig, ax = plt.subplots(figsize=(8, 5))
    im = ax.imshow(data, origin="lower", cmap="viridis", aspect="auto")
    ax.set_title(f"{LAYER_TITLES.get(layer_key, layer_key)} — {sid}")
    ax.set_xlabel("grid col")
    ax.set_ylabel("grid row")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.text(
        0.5,
        0.01,
        "Replay-side spatial concentration — explanatory only",
        ha="center",
        fontsize=8,
        color="#555",
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)
    return {"skipped": False, "path": str(out_path), "layer": layer_key, "render_profile": RENDER_PROFILE}


def render_cohort_strip(manifest: dict[str, Any], out_path: Path) -> dict[str, Any]:
    sid = manifest["sweep_id"]
    members = manifest.get("members") or []
    labels: list[str] = []
    values: list[int] = []
    for m in members:
        pid = primary_pattern(m.get("replay_pattern_tags") or []) or "other"
        labels.append(str(m.get("member_id", ""))[-12:])
        values.append(list(PATTERN_LABELS.keys()).index(pid) + 1 if pid in PATTERN_LABELS else 0)

    fig, ax = plt.subplots(figsize=(max(6, len(labels) * 1.2), 2.5))
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(labels), 1)))
    ax.bar(range(len(labels)), values or [0], color=colors[: len(labels)])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax.set_yticks([])
    ax.set_title(f"Cohort pattern strip — {sid}")
    fig.text(0.5, 0.01, "Pattern ordinal for visualization only — not scoring", ha="center", fontsize=8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)
    return {"skipped": False, "path": str(out_path), "render_profile": RENDER_PROFILE}


def render_cross_sweep_comparison_grid(
    manifests: list[dict[str, Any]],
    layer_key: str,
    out_path: Path,
) -> dict[str, Any]:
    n = len(manifests)
    cols = 2
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(10, 4 * rows))
    axes_flat = np.array(axes).flatten() if n > 1 else [axes]

    for ax, manifest in zip(axes_flat, manifests):
        sid = manifest["sweep_id"]
        spatial = manifest.get("spatial_aggregate") or {}
        grid = spatial.get("grid") or {}
        layer = (spatial.get("layers") or {}).get(layer_key) or {}
        counts = layer.get("counts") or []
        if counts:
            data = _counts_to_grid(counts, grid)
            im = ax.imshow(data, origin="lower", cmap="viridis", aspect="auto")
            ax.set_title(sid, fontsize=9)
        else:
            ax.text(0.5, 0.5, "no data", ha="center", va="center")
            im = None
        ax.set_xticks([])
        ax.set_yticks([])

    for ax in axes_flat[len(manifests) :]:
        ax.axis("off")

    fig.suptitle(f"Cross-sweep {LAYER_TITLES.get(layer_key, layer_key)}", fontsize=11)
    fig.text(0.5, 0.01, "Explanatory replay concentration — not operational prediction", ha="center", fontsize=8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)
    return {"skipped": False, "path": str(out_path), "layer": layer_key, "render_profile": RENDER_PROFILE}


def export_sweep_figures(sweep_id: str) -> list[dict[str, Any]]:
    manifest = _load_sweep(sweep_id)
    results: list[dict[str, Any]] = []
    sweep_fig_dir = _REPO / "fixtures/sa_r0/sweeps" / sweep_id / "reports" / "figures"
    pub_fig_dir = _REPO / "platform/sa-r0-viewer/public/demo/sweeps" / sweep_id / "reports" / "figures"

    for layer in ("ambiguity_density", "los_degraded", "topology_sensitivity"):
        fname = f"{sweep_id}_{layer.replace('_density', '_hotspot').replace('_degraded', '_degraded').replace('_sensitivity', '_sensitivity')}.png"
        if layer == "ambiguity_density":
            fname = f"{sweep_id}_ambiguity_hotspot.png"
        elif layer == "los_degraded":
            fname = f"{sweep_id}_los_degraded.png"
        elif layer == "topology_sensitivity":
            fname = f"{sweep_id}_topology_sensitivity.png"
        for d in (sweep_fig_dir, pub_fig_dir):
            results.append(render_layer_heatmap(manifest, layer, d / fname))

    cohort_name = f"{sweep_id}_cohort_strip.png"
    for d in (sweep_fig_dir, pub_fig_dir):
        results.append(render_cohort_strip(manifest, d / cohort_name))
    return results


def export_all_figures() -> None:
    manifests = [_load_sweep(sid) for sid in SWEEP_IDS]
    synth_dir = _REPO / "fixtures/sa_r0/synthesis/figures"
    pub_synth = _REPO / "platform/sa-r0-viewer/public/demo/synthesis/figures"
    for sid in SWEEP_IDS:
        export_sweep_figures(sid)
    for layer, fname in (
        ("ambiguity_density", "cross_sweep_comparison_grid.png"),
    ):
        for d in (synth_dir, pub_synth):
            render_cross_sweep_comparison_grid(manifests, layer, d / fname)
    print("sweep figures OK")


def main() -> None:
    export_all_figures()


if __name__ == "__main__":
    main()
