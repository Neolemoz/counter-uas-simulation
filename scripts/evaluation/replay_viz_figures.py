"""Deterministic static figure generation for replay visualization."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from replay_viz_comprehension import CATEGORY_DISPLAY_LABELS  # noqa: E402
from replay_viz_comprehension import salient_events_for_annotation  # noqa: E402

matplotlib.rcParams["svg.hashsalt"] = "replay_static_viz_comprehension_r1_v1"

RENDER_PROFILE = "static_viz_comprehension_r1_v1"
FIG_DPI = 120
FIG_WIDTH = 10.0
FIG_HEIGHT_TIMELINE = 4.0
FIG_HEIGHT_STRIP = 2.0
FIG_HEIGHT_SERIES = 3.0
FIG_HEIGHT_PANEL_ROW = 2.2

CATEGORY_COLORS: dict[str, str] = {
    "detection": "#4C78A8",
    "selection": "#F58518",
    "commitment": "#E45756",
    "ambiguity": "#72B7B2",
    "lifecycle": "#54A24B",
    "divergence": "#B279A2",
    "outcome": "#FF9DA6",
    "provenance_warning": "#9D755D",
}

CATEGORY_ORDER = [
    "detection",
    "selection",
    "commitment",
    "ambiguity",
    "lifecycle",
    "divergence",
    "outcome",
    "provenance_warning",
]

_WINDOW_FILL = {
    "divergence_mismatch_window": "#B279A233",
    "ambiguity_fragmented_gap": "#72B7B244",
    "ambiguity_fragmented_gap_open": "#72B7B222",
}

_WINDOW_LEGEND_LABELS = {
    "divergence_mismatch_window": "Divergence mismatch window",
    "ambiguity_fragmented_gap": "Fragmented gap window",
    "ambiguity_fragmented_gap_open": "Open fragmented gap",
}

_METRICS_ROW_RE = re.compile(
    r"id=(?P<iid>\S+)"
    r".*?dist=(?P<dist>[0-9]+(?:\.[0-9]+)?)\s*m"
    r".*?t_go=(?P<tgo>[0-9]+(?:\.[0-9]+)?|n/a)\s*"
    r".*?vel=(?P<vel>[0-9]+(?:\.[0-9]+)?)\s*m/s"
)
_P_HEATMAP_POS_RE = re.compile(
    r"\[P_HEATMAP\]\s+pos=\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)"
)
_GUIDANCE_POS_RE = re.compile(
    r"interceptor_pos=\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)"
    r".*target_pos=\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)"
)
_LAUNCH_ARG_RE = re.compile(r"(\w+):=(-?\d+(?:\.\d+)?)")
_STRIP_LAUNCH_PREFIX = re.compile(r"^\[[^\]]+\]\s+")


def _event_x(event: dict[str, Any], fallback: int) -> float:
    line_index = event.get("line_index")
    if isinstance(line_index, int):
        return float(line_index)
    time_s = event.get("time_s")
    if isinstance(time_s, (int, float)):
        return float(time_s)
    return float(fallback)


def _sorted_events(narrative: dict[str, Any]) -> list[dict[str, Any]]:
    events = list(narrative.get("events") or [])
    return sorted(
        events,
        key=lambda e: (
            _event_x(e, 10**9),
            str(e.get("event_id") or ""),
        ),
    )


def compute_x_limits(narrative: dict[str, Any], *, padding: float = 1.0) -> tuple[float, float]:
    """Shared horizontal limits from events and windows."""

    xs: list[float] = []
    for event in _sorted_events(narrative):
        xs.append(_event_x(event, 0))
    for window in narrative.get("windows") or []:
        start = window.get("start_line_index")
        end = window.get("end_line_index")
        if isinstance(start, int):
            xs.append(float(start))
        if isinstance(end, int):
            xs.append(float(end))
        elif isinstance(start, int):
            xs.append(float(start) + 1.0)
    if not xs:
        return 0.0, 10.0
    x_min = min(xs) - padding
    x_max = max(xs) + padding
    if x_max <= x_min:
        x_max = x_min + 1.0
    return x_min, x_max


def _category_y_labels() -> list[str]:
    return [CATEGORY_DISPLAY_LABELS.get(cat, cat.replace("_", " ").title()) for cat in CATEGORY_ORDER]


def _new_figure(*, width: float, height: float) -> tuple[plt.Figure, plt.Axes]:
    plt.close("all")
    fig, ax = plt.subplots(num=1, clear=True, figsize=(width, height))
    return fig, ax


def _save_figure(fig: plt.Figure, out_path: Path, *, fmt: str = "png") -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, format=fmt, dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)
    plt.close("all")


def _apply_window_shading(
    ax: plt.Axes,
    windows: list[dict[str, Any]],
    y_lo: float,
    y_hi: float,
    *,
    legend_seen: set[str] | None = None,
) -> set[str]:
    seen = legend_seen if legend_seen is not None else set()
    for window in windows:
        start = window.get("start_line_index")
        end = window.get("end_line_index")
        if not isinstance(start, int):
            continue
        if not isinstance(end, int):
            end = start + 1
        window_type = str(window.get("window_type") or "")
        fill = _WINDOW_FILL.get(window_type, "#CCCCCC22")
        legend_label = _WINDOW_LEGEND_LABELS.get(window_type, window_type)
        label = legend_label if window_type and window_type not in seen else None
        if label:
            seen.add(window_type)
        ax.axvspan(float(start), float(end), ymin=y_lo, ymax=y_hi, color=fill, linewidth=0, label=label)
    return seen


def _short_annotation(label: str, *, max_len: int = 28) -> str:
    text = label.strip()
    if len(text) <= max_len:
        return text
    return text[: max_len - 1] + "…"


def _annotate_salient_events(ax: plt.Axes, narrative: dict[str, Any], category_to_y: dict[str, int]) -> None:
    events = _sorted_events(narrative)
    for event in salient_events_for_annotation(events):
        category = str(event.get("category") or "lifecycle")
        y = category_to_y.get(category, category_to_y.get("lifecycle", 0))
        x = _event_x(event, 0)
        label = _short_annotation(str(event.get("label") or ""))
        if not label:
            continue
        ax.annotate(
            label,
            (x, y),
            textcoords="offset points",
            xytext=(0, 8),
            fontsize=7,
            ha="center",
            alpha=0.9,
        )


def _draw_timeline_on_axes(
    ax: plt.Axes,
    narrative: dict[str, Any],
    *,
    x_limits: tuple[float, float] | None = None,
    annotate: bool = True,
    show_legend: bool = True,
) -> list[str]:
    events = _sorted_events(narrative)
    category_to_y = {cat: idx for idx, cat in enumerate(CATEGORY_ORDER)}
    source_event_ids: list[str] = []

    for idx, event in enumerate(events):
        category = str(event.get("category") or "lifecycle")
        y = category_to_y.get(category, category_to_y["lifecycle"])
        x = _event_x(event, idx + 1)
        color = CATEGORY_COLORS.get(category, "#888888")
        ax.scatter([x], [y], c=color, s=36, zorder=3, edgecolors="white", linewidths=0.5)
        event_id = event.get("event_id")
        if event_id:
            source_event_ids.append(str(event_id))

    legend_seen: set[str] = set()
    _apply_window_shading(ax, list(narrative.get("windows") or []), 0.0, 1.0, legend_seen=legend_seen)
    if annotate:
        _annotate_salient_events(ax, narrative, category_to_y)

    ax.set_yticks(list(range(len(CATEGORY_ORDER))))
    ax.set_yticklabels(_category_y_labels(), fontsize=8)
    ax.set_xlabel("Replay line index (or ordinal when line unavailable)", fontsize=8)
    ax.set_ylim(-0.5, len(CATEGORY_ORDER) - 0.5)
    if x_limits is not None:
        ax.set_xlim(x_limits)
    ax.grid(True, axis="x", alpha=0.25, linestyle="--")
    if show_legend and legend_seen:
        ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    return source_event_ids


def _draw_divergence_on_axes(
    ax: plt.Axes,
    narrative: dict[str, Any],
    *,
    x_limits: tuple[float, float] | None = None,
    show_legend: bool = True,
) -> list[str]:
    events = _sorted_events(narrative)
    windows = list(narrative.get("windows") or [])
    mismatch_events = [
        e for e in events if e.get("event_type") in ("selection_oracle_mismatch", "selection_block")
    ]
    source_event_ids: list[str] = []
    legend_seen: set[str] = set()

    for window in windows:
        start = window.get("start_line_index")
        end = window.get("end_line_index")
        if not isinstance(start, int):
            continue
        if not isinstance(end, int):
            end = start + 1
        window_type = str(window.get("window_type") or "")
        fill = _WINDOW_FILL.get(window_type, "#B279A233")
        legend_label = _WINDOW_LEGEND_LABELS.get(window_type, window_type)
        label = legend_label if window_type and window_type not in legend_seen else None
        if label:
            legend_seen.add(window_type)
        ax.axvspan(float(start), float(end), color=fill, label=label)

    mismatch_labeled = False
    selection_labeled = False
    for event in mismatch_events:
        x = _event_x(event, 0)
        is_mismatch = event.get("event_type") == "selection_oracle_mismatch"
        color = CATEGORY_COLORS["divergence"] if is_mismatch else CATEGORY_COLORS["selection"]
        label = None
        if is_mismatch and not mismatch_labeled:
            label = "Oracle mismatch marker"
            mismatch_labeled = True
        elif not is_mismatch and not selection_labeled:
            label = "Selection block marker"
            selection_labeled = True
        ax.axvline(x, color=color, linestyle="--" if is_mismatch else "-", linewidth=1.2, alpha=0.85, label=label)
        event_id = event.get("event_id")
        if event_id:
            source_event_ids.append(str(event_id))

    ax.set_ylim(0.0, 1.0)
    ax.set_yticks([])
    ax.set_xlabel("Replay line index", fontsize=8)
    if x_limits is not None:
        ax.set_xlim(x_limits)
    ax.grid(True, axis="x", alpha=0.25, linestyle="--")
    if show_legend and (legend_seen or mismatch_labeled or selection_labeled):
        ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    return source_event_ids


def _draw_lifecycle_on_axes(
    ax: plt.Axes,
    narrative: dict[str, Any],
    *,
    x_limits: tuple[float, float] | None = None,
) -> list[str]:
    events = [e for e in _sorted_events(narrative) if str(e.get("category")) == "lifecycle"]
    source_event_ids: list[str] = []

    for idx, event in enumerate(events):
        x = _event_x(event, idx + 1)
        ax.scatter([x], [0.5], c=CATEGORY_COLORS["lifecycle"], s=40, zorder=3)
        event_id = event.get("event_id")
        if event_id:
            source_event_ids.append(str(event_id))

    _apply_window_shading(ax, list(narrative.get("windows") or []), 0.0, 1.0, legend_seen=set())
    ax.set_ylim(0.0, 1.0)
    ax.set_yticks([])
    ax.set_xlabel("Replay line index", fontsize=8)
    if x_limits is not None:
        ax.set_xlim(x_limits)
    ax.grid(True, axis="x", alpha=0.25, linestyle="--")
    return source_event_ids


def render_timeline_band(narrative: dict[str, Any], out_path: Path) -> dict[str, Any]:
    """Render categorical event timeline band."""

    x_limits = compute_x_limits(narrative)
    fig, ax = _new_figure(width=FIG_WIDTH, height=FIG_HEIGHT_TIMELINE)
    source_event_ids = _draw_timeline_on_axes(ax, narrative, x_limits=x_limits)
    ax.set_title(
        "Replay event timeline by category\n(explanatory visualization layer; not causal proof)",
        fontsize=10,
    )

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "timeline_band_0001",
        "category": "timeline_band",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": sorted(source_event_ids),
        "interpretation_caveat": (
            "Timeline band is reviewer-facing sequence context only; not causal proof or unified replay state."
        ),
    }


def render_divergence_overlay(narrative: dict[str, Any], out_path: Path) -> dict[str, Any]:
    """Render divergence and ambiguity window overlay with selection markers."""

    x_limits = compute_x_limits(narrative)
    fig, ax = _new_figure(width=FIG_WIDTH, height=FIG_HEIGHT_STRIP + 0.6)
    source_event_ids = _draw_divergence_on_axes(ax, narrative, x_limits=x_limits)
    ax.set_title(
        "Divergence and selection localization\n(non-causal; replay-side disagreement only)",
        fontsize=10,
    )

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "divergence_overlay_0001",
        "category": "divergence_overlay",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": sorted(source_event_ids),
        "interpretation_caveat": (
            "Divergence shading localizes replay-side disagreement; does not replace tactical authority surfaces."
        ),
    }


def render_lifecycle_strip(narrative: dict[str, Any], out_path: Path) -> dict[str, Any]:
    """Render lifecycle event strip from narrative lifecycle-category events."""

    x_limits = compute_x_limits(narrative)
    fig, ax = _new_figure(width=FIG_WIDTH, height=FIG_HEIGHT_STRIP + 0.4)
    source_event_ids = _draw_lifecycle_on_axes(ax, narrative, x_limits=x_limits)
    ax.set_title(
        "Lifecycle evidence strip\n(explanatory overlay; not tracker lifecycle truth)",
        fontsize=10,
    )

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "lifecycle_strip_0001",
        "category": "lifecycle_strip",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": sorted(source_event_ids),
        "interpretation_caveat": (
            "Lifecycle strip is an explanatory overlay over raw log lines; not robustness proof."
        ),
    }


def render_comprehension_panel(narrative: dict[str, Any], out_path: Path) -> dict[str, Any]:
    """Render stacked timeline, divergence, and lifecycle panels with shared x-axis."""

    x_limits = compute_x_limits(narrative)
    plt.close("all")
    fig, axes = plt.subplots(
        3,
        1,
        num=1,
        clear=True,
        figsize=(FIG_WIDTH, FIG_HEIGHT_PANEL_ROW * 3 + 0.8),
        sharex=True,
    )
    all_ids: list[str] = []
    all_ids.extend(_draw_timeline_on_axes(axes[0], narrative, x_limits=x_limits, annotate=False, show_legend=False))
    axes[0].set_title("Timeline", fontsize=9, loc="left")
    all_ids.extend(_draw_divergence_on_axes(axes[1], narrative, x_limits=x_limits, show_legend=False))
    axes[1].set_title("Divergence localization", fontsize=9, loc="left")
    all_ids.extend(_draw_lifecycle_on_axes(axes[2], narrative, x_limits=x_limits))
    axes[2].set_title("Lifecycle evidence", fontsize=9, loc="left")
    fig.suptitle(
        "Stacked replay overview (explanatory; shared log index; not unified replay state)",
        fontsize=10,
        y=1.02,
    )
    fig.tight_layout()

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "comprehension_panel_0001",
        "category": "comprehension_panel",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": sorted(set(all_ids)),
        "interpretation_caveat": (
            "Stacked panel localizes replay evidence along a shared log index; not causal proof or operational truth."
        ),
    }


def _strip_ros_prefix(line: str) -> str:
    return _STRIP_LAUNCH_PREFIX.sub("", line.strip())


def parse_engagement_series(log_path: Path) -> dict[str, list[float]] | None:
    if not log_path.is_file():
        return None
    text = log_path.read_text(encoding="utf-8", errors="replace")
    dist_series: list[float] = []
    tgo_series: list[float] = []
    for raw_line in text.splitlines():
        s = _strip_ros_prefix(raw_line)
        match = _METRICS_ROW_RE.search(s)
        if match and "dist=" in s and "t_go=" in s:
            dist_series.append(round(float(match.group("dist")), 3))
            tgo_raw = match.group("tgo")
            if tgo_raw != "n/a":
                tgo_series.append(round(float(tgo_raw), 2))
    if not dist_series:
        return None
    return {"dist_m": dist_series, "t_go_s": tgo_series}


def render_engagement_series(log_path: Path, out_path: Path) -> dict[str, Any] | None:
    series = parse_engagement_series(log_path)
    if series is None:
        return None

    plt.close("all")
    fig = plt.figure(num=1, clear=True, figsize=(FIG_WIDTH, FIG_HEIGHT_SERIES * 1.4))
    ax_dist = fig.add_subplot(2, 1, 1)
    ax_tgo = fig.add_subplot(2, 1, 2)
    x_dist = list(range(len(series["dist_m"])))
    ax_dist.plot(x_dist, series["dist_m"], color=CATEGORY_COLORS["outcome"], linewidth=1.5)
    ax_dist.set_ylabel("dist (m)")
    ax_dist.set_title("Engagement distance from [METRICS] (log-evidenced samples only)")
    ax_dist.grid(True, alpha=0.25, linestyle="--")

    if series["t_go_s"]:
        x_tgo = list(range(len(series["t_go_s"])))
        ax_tgo.plot(x_tgo, series["t_go_s"], color=CATEGORY_COLORS["selection"], linewidth=1.5)
    ax_tgo.set_ylabel("t_go (s)")
    ax_tgo.set_xlabel("Sample index (not wall-clock replay time)")
    ax_tgo.grid(True, alpha=0.25, linestyle="--")

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "engagement_series_0001",
        "category": "engagement_series",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": [],
        "interpretation_caveat": (
            "Engagement series is sparse log-evidenced sampling only; not continuous trajectory authority."
        ),
    }


def _parse_launch_geometry(meta_path: Path | None, cmd: Any) -> tuple[float, float] | None:
    text_parts: list[str] = []
    if isinstance(cmd, list):
        text_parts.append(" ".join(str(c) for c in cmd))
    elif isinstance(cmd, str):
        text_parts.append(cmd)
    if meta_path and meta_path.is_file():
        try:
            import json

            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            if isinstance(meta.get("cmd"), list):
                text_parts.append(" ".join(str(c) for c in meta["cmd"]))
        except (OSError, json.JSONDecodeError):
            pass
    for part in text_parts:
        args = dict(_LAUNCH_ARG_RE.findall(part))
        if "target_start_x_m" in args and "target_start_y_m" in args:
            return round(float(args["target_start_x_m"]), 3), round(float(args["target_start_y_m"]), 3)
    return None


def parse_sparse_xy_points(log_path: Path, meta_path: Path | None = None) -> list[dict[str, Any]]:
    if not log_path.is_file():
        return []
    points: list[dict[str, Any]] = []
    text = log_path.read_text(encoding="utf-8", errors="replace")
    for raw_line in text.splitlines():
        s = _strip_ros_prefix(raw_line)
        heat = _P_HEATMAP_POS_RE.search(s)
        if heat:
            points.append(
                {
                    "label": "p_heatmap",
                    "x_m": round(float(heat.group(1)), 3),
                    "y_m": round(float(heat.group(2)), 3),
                }
            )
            continue
        guide = _GUIDANCE_POS_RE.search(s)
        if guide:
            points.append(
                {
                    "label": "interceptor",
                    "x_m": round(float(guide.group(1)), 3),
                    "y_m": round(float(guide.group(2)), 3),
                }
            )
            points.append(
                {
                    "label": "target",
                    "x_m": round(float(guide.group(4)), 3),
                    "y_m": round(float(guide.group(5)), 3),
                }
            )
    start = _parse_launch_geometry(meta_path, None)
    if start is not None:
        points.append({"label": "launch_start", "x_m": start[0], "y_m": start[1]})
    return points


def render_sparse_topdown(
    log_path: Path,
    out_path: Path,
    *,
    meta_path: Path | None = None,
) -> dict[str, Any] | None:
    points = parse_sparse_xy_points(log_path, meta_path=meta_path)
    if not points:
        return None

    fig, ax = _new_figure(width=FIG_WIDTH, height=FIG_WIDTH)
    label_colors = {
        "p_heatmap": CATEGORY_COLORS["ambiguity"],
        "interceptor": CATEGORY_COLORS["selection"],
        "target": CATEGORY_COLORS["detection"],
        "launch_start": CATEGORY_COLORS["outcome"],
    }
    xs = [float(p["x_m"]) for p in points]
    ys = [float(p["y_m"]) for p in points]
    if len(xs) >= 2:
        ax.plot(
            xs,
            ys,
            color="#888888",
            linestyle="--",
            linewidth=0.8,
            alpha=0.5,
            label="log sample sequence (not continuous path)",
            zorder=1,
        )

    for point in points:
        label = str(point.get("label") or "point")
        ax.scatter(
            [point["x_m"]],
            [point["y_m"]],
            c=label_colors.get(label, "#888888"),
            s=48,
            label=label,
            zorder=3,
        )

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        by_label = dict(zip(labels, handles, strict=False))
        ax.legend(by_label.values(), by_label.keys(), loc="best", fontsize=8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(
        "Sparse log-evidenced XY samples\n(not ground-truth trajectory; not operational geometry)",
        fontsize=10,
    )
    ax.grid(True, alpha=0.25, linestyle="--")

    _save_figure(fig, out_path, fmt=out_path.suffix.lstrip(".") or "png")
    return {
        "figure_id": "sparse_topdown_0001",
        "category": "sparse_topdown",
        "format": out_path.suffix.lstrip(".") or "png",
        "path": str(out_path),
        "source_event_ids": [],
        "interpretation_caveat": (
            "Sparse top-down view shows log-evidenced samples only; not continuous ground-truth trajectory."
        ),
    }
