"""Reviewer comprehension digest derived from frozen replay narrative JSON."""

from __future__ import annotations

from typing import Any

FIGURE_DISPLAY_ORDER = [
    "timeline_band",
    "divergence_overlay",
    "lifecycle_strip",
    "comprehension_panel",
    "sparse_topdown",
    "engagement_series",
]

CATEGORY_DISPLAY_LABELS: dict[str, str] = {
    "detection": "Detection",
    "selection": "Selection",
    "commitment": "Commitment",
    "ambiguity": "Ambiguity",
    "lifecycle": "Lifecycle",
    "divergence": "Divergence",
    "outcome": "Outcome",
    "provenance_warning": "Provenance warning",
}

CATEGORY_INCIDENT_ORDER = [
    "divergence",
    "selection",
    "ambiguity",
    "detection",
    "lifecycle",
    "outcome",
    "provenance_warning",
    "commitment",
]

WINDOW_TYPE_LABELS: dict[str, str] = {
    "divergence_mismatch_window": "Divergence mismatch window",
    "ambiguity_fragmented_gap": "Fragmented gap window",
    "ambiguity_fragmented_gap_open": "Open fragmented gap window",
}

FIGURE_REVIEWER_TITLES: dict[str, str] = {
    "timeline_band": "Event timeline by category",
    "divergence_overlay": "Divergence and selection localization",
    "lifecycle_strip": "Lifecycle evidence strip",
    "comprehension_panel": "Stacked replay overview",
    "sparse_topdown": "Sparse log-evidenced positions",
    "engagement_series": "Engagement metrics samples",
}

FIGURE_LOOK_FOR: dict[str, str] = {
    "timeline_band": "Locate when each evidence category appears along the replay log index.",
    "divergence_overlay": "Find shaded windows and vertical markers where selection evidence disagrees with oracle replay.",
    "lifecycle_strip": "See lifecycle-category log evidence localized in time; not tracker lifecycle truth.",
    "comprehension_panel": "Scan timeline, divergence, and lifecycle panels with shared horizontal scale.",
    "sparse_topdown": "Review isolated XY samples only; dashed sequence is not a continuous path.",
    "engagement_series": "Review sparse [METRICS] samples; not continuous engagement authority.",
}

_ANNOTATION_PRIORITY: dict[str, int] = {
    "selection_oracle_mismatch": 0,
    "provenance_warning": 1,
    "divergence_class": 2,
    "selection_block": 3,
    "realism_fragmented_gap_start": 4,
    "candidate_spawn": 5,
}


def _event_sort_key(event: dict[str, Any]) -> tuple[Any, ...]:
    line_index = event.get("line_index")
    time_s = event.get("time_s")
    return (
        line_index is None,
        line_index if isinstance(line_index, int) else 10**9,
        time_s is None,
        time_s if isinstance(time_s, (int, float)) else 0.0,
        str(event.get("event_id") or ""),
    )


def _sorted_events(narrative: dict[str, Any]) -> list[dict[str, Any]]:
    return sorted(list(narrative.get("events") or []), key=_event_sort_key)


def _event_by_id(narrative: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(e.get("event_id")): e for e in _sorted_events(narrative) if e.get("event_id")}


def _format_line_index(line_index: Any) -> str:
    if isinstance(line_index, int):
        return str(line_index)
    return "—"


def build_comprehension_digest(narrative: dict[str, Any]) -> dict[str, Any]:
    """Build deterministic reviewer comprehension fields from narrative JSON."""

    summary = narrative.get("summary") if isinstance(narrative.get("summary"), dict) else {}
    lineage = narrative.get("lineage") if isinstance(narrative.get("lineage"), dict) else {}
    events = _sorted_events(narrative)
    windows = list(narrative.get("windows") or [])
    warnings = list(narrative.get("warnings") or [])
    event_by_id = _event_by_id(narrative)

    run_id = summary.get("run_id") or lineage.get("run_id") or "unknown"
    divergence_class = summary.get("selection_oracle_divergence_class") or "not recorded"
    event_count = summary.get("event_count", len(events))
    window_count = summary.get("window_count", len(windows))

    headline = (
        f"Derived replay summary for {run_id}: "
        f"{event_count} narrative events, {window_count} localized windows, "
        f"divergence class {divergence_class}."
    )

    scan_guide = [
        "This report is an explanatory visualization layer only; it does not replace parser-visible summaries or runtime authority.",
        "Start with Key incidents and At a glance, then use Figures to localize evidence along the replay log index.",
        "Divergence shading and labels localize replay-side disagreement; they are not causal proof.",
        "Sparse position and engagement panels show log-evidenced samples only when present; they are not continuous trajectories.",
        "Read Warnings and Interpretation caveats before drawing conclusions about readiness or robustness.",
    ]

    canonical = summary.get("canonical_parser_visible_summary")
    if not isinstance(canonical, dict):
        canonical = {}
    hit = canonical.get("hit")
    hit_label = "hit recorded" if hit is True else "no hit recorded" if hit is False else "hit status unavailable"

    at_a_glance = [
        {"label": "Run", "value": str(run_id)},
        {"label": "Divergence class", "value": str(divergence_class)},
        {"label": "Parser-visible outcome", "value": hit_label},
        {"label": "Narrative events", "value": str(event_count)},
        {"label": "Localized windows", "value": str(window_count)},
        {"label": "Warnings", "value": str(len(warnings))},
    ]

    incident_groups: list[dict[str, Any]] = []
    by_category: dict[str, list[dict[str, Any]]] = {}
    for event in events:
        category = str(event.get("category") or "lifecycle")
        by_category.setdefault(category, []).append(
            {
                "label": str(event.get("label") or event.get("event_type") or ""),
                "time_label": str(event.get("time_label") or "time unavailable"),
                "line_index": _format_line_index(event.get("line_index")),
            }
        )

    for category in CATEGORY_INCIDENT_ORDER:
        rows = by_category.get(category)
        if not rows:
            continue
        incident_groups.append(
            {
                "category": category,
                "category_label": CATEGORY_DISPLAY_LABELS.get(category, category.replace("_", " ").title()),
                "rows": rows,
            }
        )

    key_windows: list[dict[str, str]] = []
    for window in windows:
        window_type = str(window.get("window_type") or "")
        start_id = window.get("start_event_id")
        start_label = ""
        if start_id and str(start_id) in event_by_id:
            start_label = str(event_by_id[str(start_id)].get("label") or "")
        key_windows.append(
            {
                "window_type": window_type,
                "window_label": WINDOW_TYPE_LABELS.get(window_type, window_type.replace("_", " ")),
                "start_line_index": _format_line_index(window.get("start_line_index")),
                "end_line_index": _format_line_index(window.get("end_line_index")),
                "start_event_label": start_label,
            }
        )

    timeline_rows: list[dict[str, str]] = []
    for event in events:
        timeline_rows.append(
            {
                "time_label": str(event.get("time_label") or "time unavailable"),
                "category_label": CATEGORY_DISPLAY_LABELS.get(
                    str(event.get("category") or ""),
                    str(event.get("category") or ""),
                ),
                "label": str(event.get("label") or ""),
                "line_index": _format_line_index(event.get("line_index")),
            }
        )

    return {
        "at_a_glance": at_a_glance,
        "figure_display_order": list(FIGURE_DISPLAY_ORDER),
        "figure_look_for": dict(FIGURE_LOOK_FOR),
        "figure_reviewer_titles": dict(FIGURE_REVIEWER_TITLES),
        "headline": headline,
        "incident_groups": incident_groups,
        "key_windows": key_windows,
        "scan_guide": scan_guide,
        "timeline_rows": timeline_rows,
    }


def salient_events_for_annotation(events: list[dict[str, Any]], *, limit: int = 8) -> list[dict[str, Any]]:
    """Return up to `limit` events for timeline annotations, deterministically prioritized."""

    def priority(event: dict[str, Any]) -> tuple[Any, ...]:
        event_type = str(event.get("event_type") or "")
        return (
            _ANNOTATION_PRIORITY.get(event_type, 99),
            _event_sort_key(event),
        )

    candidates = [
        e
        for e in events
        if str(e.get("event_type") or "") in _ANNOTATION_PRIORITY
        or str(e.get("category") or "") in ("divergence", "provenance_warning")
    ]
    return sorted(candidates, key=priority)[:limit]
