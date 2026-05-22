"""Reviewer comprehension digest derived from frozen replay narrative JSON."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

FIGURE_DISPLAY_ORDER = [
    "timeline_band",
    "divergence_overlay",
    "lifecycle_strip",
    "comprehension_panel",
    "sparse_topdown",
    "engagement_series",
]

TIMELINE_SALIENT_LIMIT = 40
MISMATCH_INCIDENT_LIMIT = 5
NEAR_THRESHOLD_MARGIN_M = 0.25

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

DIVERGENCE_CLASS_LABELS: dict[str, str] = {
    "D0_no_divergence": "No divergence (D0)",
    "D1_oracle_mismatch_only": "Oracle mismatch only (D1)",
    "D2_mismatch_after_fragmented_gap": "Mismatch after fragmented gap (D2)",
    "D3_persistent_mismatch": "Persistent mismatch (D3)",
    "D4_ambiguous_selection_evidence": "Ambiguous selection evidence (D4)",
    "D5_inconclusive_visibility_limited": "Inconclusive (visibility-limited evidence)",
}

DIVERGENCE_D5_COEXISTENCE_NOTE = (
    "D5 means replay evidence was insufficient for a stronger divergence class; "
    "localized mismatch markers may still appear in specific selection blocks."
)

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
    "outcome_hit": 3,
    "outcome_miss_distance": 4,
    "selection_block": 5,
    "realism_fragmented_gap_start": 6,
    "realism_fragmented_gap_end": 7,
    "candidate_spawn": 8,
}

_HIT_THR_RE = re.compile(r"hit_threshold\s*=\s*(?P<thr>[0-9]+(?:\.[0-9]+)?)\s*m")
_MIN_MISS_RE = re.compile(r"\[min_miss\]\s*=\s*(?P<mm>[0-9]+(?:\.[0-9]+)?)\s*m\b")
_MIN_MISS_IN_HIT_RE = re.compile(r"\bmin_miss=(?P<mm>[0-9]+(?:\.[0-9]+)?)\s*m\b")

_VISIBILITY_LIMITED_WARNING = "divergence classification is visibility-limited"


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


def _maybe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_log_outcome_metrics(log_path: Path | str | None) -> dict[str, float | None]:
    """Parse log-evidenced min_miss and hit_threshold (non-authoritative)."""

    if not log_path:
        return {"hit_threshold_m": None, "min_miss_m": None}
    path = Path(str(log_path))
    if not path.is_file():
        return {"hit_threshold_m": None, "min_miss_m": None}

    hit_threshold_m: float | None = None
    min_miss_m: float | None = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        m = _MIN_MISS_RE.search(s)
        if m:
            v = float(m.group("mm"))
            if min_miss_m is None or v < min_miss_m:
                min_miss_m = v
            continue
        m = _HIT_THR_RE.search(s)
        if m:
            hit_threshold_m = float(m.group("thr"))
        m = _MIN_MISS_IN_HIT_RE.search(s)
        if m:
            v = float(m.group("mm"))
            if min_miss_m is None or v < min_miss_m:
                min_miss_m = v
    return {"hit_threshold_m": hit_threshold_m, "min_miss_m": min_miss_m}


def _evaluation_row_from_observability(observability: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(observability, dict):
        return {}
    bundle = observability.get("bundle")
    if not isinstance(bundle, dict):
        bundle = observability
    layers = bundle.get("evidence_layers") if isinstance(bundle.get("evidence_layers"), dict) else {}
    derived = layers.get("derived_evaluation_artifacts") if isinstance(layers.get("derived_evaluation_artifacts"), dict) else {}
    row = derived.get("evaluation_row")
    return row if isinstance(row, dict) else {}


def _trace_summary_from_observability(observability: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(observability, dict):
        return {}
    trace = observability.get("divergence_trace")
    if not isinstance(trace, dict):
        return {}
    summary = trace.get("summary")
    return summary if isinstance(summary, dict) else {}


def human_divergence_class_label(divergence_class: str) -> str:
    if not divergence_class or divergence_class == "not recorded":
        return "not recorded"
    return DIVERGENCE_CLASS_LABELS.get(divergence_class, divergence_class.replace("_", " "))


def display_outcome_hit_label(*, hit: Any) -> str:
    if hit is True:
        return "Parser-visible hit recorded (explanatory copy of existing summary)"
    if hit is False:
        return "Parser-visible miss recorded (no hit in summary)"
    return "Parser-visible hit status unavailable"


def display_miss_distance_label(miss_distance_m: Any) -> str:
    if miss_distance_m is None:
        return "Minimum miss distance unavailable"
    return f"Minimum miss distance recorded: {miss_distance_m} m (parser-visible)"


def display_event_label(event: dict[str, Any], canonical: dict[str, Any]) -> str:
    """Normalize reviewer-facing labels without mutating frozen narrative JSON."""

    event_type = str(event.get("event_type") or "")
    details = event.get("details") if isinstance(event.get("details"), dict) else {}

    if event_type == "outcome_hit":
        hit = details.get("hit")
        if hit is None:
            hit = canonical.get("hit")
        return display_outcome_hit_label(hit=hit)
    if event_type == "outcome_miss_distance":
        return display_miss_distance_label(details.get("miss_distance_m"))
    if event_type == "divergence_class":
        raw = str(details.get("selection_oracle_divergence_class") or "")
        human = human_divergence_class_label(raw)
        if raw and human != raw:
            return f"Divergence class: {human} ({raw})"
        return str(event.get("label") or "")
    if event_type == "selection_oracle_mismatch":
        block = details.get("block_index")
        return f"Selection/oracle mismatch localized at block {block}"
    if event_type == "selection_block":
        return str(event.get("label") or "")
    if event_type == "provenance_warning":
        warning = str(details.get("warning") or "")
        if warning.strip().lower() == _VISIBILITY_LIMITED_WARNING:
            return ""
    return str(event.get("label") or event.get("event_type") or "")


def _incident_row(event: dict[str, Any], canonical: dict[str, Any], *, salient: bool = False) -> dict[str, str]:
    label = display_event_label(event, canonical)
    row: dict[str, str] = {
        "time_label": str(event.get("time_label") or "time unavailable"),
        "label": label,
        "line_index": _format_line_index(event.get("line_index")),
    }
    if salient:
        row["salient"] = "true"
    return row


def _build_selection_summary(
    events: list[dict[str, Any]],
    trace_summary: dict[str, Any],
) -> dict[str, Any]:
    selection_blocks = [e for e in events if e.get("event_type") == "selection_block"]
    mismatches = [e for e in events if e.get("event_type") == "selection_oracle_mismatch"]
    block_indices = sorted(
        int(e["details"]["block_index"])
        for e in selection_blocks
        if isinstance(e.get("details"), dict) and isinstance(e["details"].get("block_index"), int)
    )
    mismatch_indices = sorted(
        int(e["details"]["block_index"])
        for e in mismatches
        if isinstance(e.get("details"), dict) and isinstance(e["details"].get("block_index"), int)
    )
    after_gap = sum(
        1
        for e in selection_blocks
        if isinstance(e.get("details"), dict) and e["details"].get("after_fragmented_gap") is True
    )

    first_mismatch = trace_summary.get("first_selection_mismatch_block")
    if first_mismatch is None and mismatch_indices:
        first_mismatch = mismatch_indices[0]
    mismatch_count = trace_summary.get("selection_mismatch_count")
    if mismatch_count is None:
        mismatch_count = len(mismatches)
    try:
        mismatch_count = int(mismatch_count)
    except (TypeError, ValueError):
        mismatch_count = len(mismatches)

    last_mismatch = mismatch_indices[-1] if mismatch_indices else None

    summary_line = "No selection blocks recorded"
    if block_indices:
        if len(block_indices) == 1:
            block_span = f"block {block_indices[0]}"
        else:
            block_span = f"blocks {block_indices[0]}–{block_indices[-1]}"
        summary_line = (
            f"{len(block_indices)} selection block(s) recorded ({block_span}); "
            "see selection detail appendix for per-block traceability"
        )

    return {
        "selection_block_count": len(block_indices),
        "mismatch_count": mismatch_count,
        "first_mismatch_block": first_mismatch,
        "last_mismatch_block": last_mismatch,
        "blocks_after_fragmented_gap": after_gap,
        "summary_line": summary_line,
    }


def _build_outcome_context(
    canonical: dict[str, Any],
    eval_row: dict[str, Any],
    log_metrics: dict[str, float | None],
) -> dict[str, Any]:
    hit = canonical.get("hit")
    miss_distance_m = _maybe_float(canonical.get("miss_distance_m"))
    if miss_distance_m is None:
        miss_distance_m = _maybe_float(eval_row.get("min_miss_m"))
    if miss_distance_m is None:
        miss_distance_m = log_metrics.get("min_miss_m")

    hit_threshold_m = _maybe_float(eval_row.get("hit_threshold_m"))
    if hit_threshold_m is None:
        hit_threshold_m = log_metrics.get("hit_threshold_m")

    margin_m: float | None = None
    near_threshold = False
    if miss_distance_m is not None and hit_threshold_m is not None:
        margin_m = round(hit_threshold_m - miss_distance_m, 4)
        near_threshold = abs(margin_m) <= NEAR_THRESHOLD_MARGIN_M

    return {
        "hit": hit,
        "outcome_label": display_outcome_hit_label(hit=hit),
        "miss_distance_m": miss_distance_m,
        "hit_threshold_m": hit_threshold_m,
        "margin_to_threshold_m": margin_m,
        "near_evaluation_threshold": near_threshold,
        "near_threshold_note": (
            "Near evaluation threshold (explanatory; not operational failure)"
            if near_threshold
            else None
        ),
    }


def _build_incident_groups(
    events: list[dict[str, Any]],
    canonical: dict[str, Any],
    selection_summary: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    """Build collapsed incident groups and selection detail appendix rows."""

    selection_detail_rows: list[dict[str, str]] = []
    by_category: dict[str, list[dict[str, str]]] = {}

    for event in events:
        category = str(event.get("category") or "lifecycle")
        event_type = str(event.get("event_type") or "")
        label = display_event_label(event, canonical)
        if not label:
            continue

        row = _incident_row(event, canonical, salient=event_type in _ANNOTATION_PRIORITY)

        if event_type == "selection_block":
            selection_detail_rows.append(row)
            continue

        if event_type == "selection_oracle_mismatch":
            by_category.setdefault("divergence", []).append(row)
            continue

        by_category.setdefault(category, []).append(row)

    selection_rows: list[dict[str, str]] = []
    if selection_summary.get("selection_block_count", 0) > 0:
        selection_rows.append(
            {
                "time_label": "summary",
                "label": str(selection_summary.get("summary_line") or ""),
                "line_index": "—",
                "salient": "true",
            }
        )

    mismatch_rows = by_category.get("divergence", [])
    mismatch_only = [r for r in mismatch_rows if "mismatch localized" in r.get("label", "")]
    other_divergence = [r for r in mismatch_rows if r not in mismatch_only]

    if len(mismatch_only) > MISMATCH_INCIDENT_LIMIT:
        shown = mismatch_only[:MISMATCH_INCIDENT_LIMIT]
        hidden = len(mismatch_only) - MISMATCH_INCIDENT_LIMIT
        shown.append(
            {
                "time_label": "summary",
                "label": f"+{hidden} additional mismatch block(s) in selection detail appendix",
                "line_index": "—",
                "salient": "true",
            }
        )
        by_category["divergence"] = shown + other_divergence
    else:
        by_category["divergence"] = mismatch_rows

    if selection_rows:
        by_category["selection"] = selection_rows

    incident_groups: list[dict[str, Any]] = []
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

    return incident_groups, selection_detail_rows


def _timeline_row(event: dict[str, Any], canonical: dict[str, Any], *, salient: bool = False) -> dict[str, str]:
    row = {
        "time_label": str(event.get("time_label") or "time unavailable"),
        "category_label": CATEGORY_DISPLAY_LABELS.get(
            str(event.get("category") or ""),
            str(event.get("category") or ""),
        ),
        "label": display_event_label(event, canonical),
        "line_index": _format_line_index(event.get("line_index")),
    }
    if salient:
        row["salient"] = "true"
    if str(event.get("category") or "") == "provenance_warning":
        row["warning"] = "true"
    return row


def _build_timeline_salient(
    events: list[dict[str, Any]],
    canonical: dict[str, Any],
    *,
    limit: int = TIMELINE_SALIENT_LIMIT,
) -> tuple[list[dict[str, str]], list[dict[str, str]], dict[str, Any]]:
    full_rows: list[dict[str, str]] = []
    for event in events:
        label = display_event_label(event, canonical)
        if label:
            full_rows.append(_timeline_row(event, canonical))

    def salience_key(event: dict[str, Any]) -> tuple[Any, ...]:
        event_type = str(event.get("event_type") or "")
        return (_ANNOTATION_PRIORITY.get(event_type, 99), _event_sort_key(event))

    salient_events = sorted(
        [
            e
            for e in events
            if display_event_label(e, canonical)
            and (
                str(e.get("event_type") or "") in _ANNOTATION_PRIORITY
                or str(e.get("category") or "") in ("divergence", "provenance_warning", "outcome")
            )
        ],
        key=salience_key,
    )

    salient_ids = {str(e.get("event_id")) for e in salient_events}
    salient_rows: list[dict[str, str]] = []
    seen_ids: set[str] = set()

    for event in salient_events:
        eid = str(event.get("event_id") or "")
        if eid in seen_ids:
            continue
        seen_ids.add(eid)
        salient_rows.append(_timeline_row(event, canonical, salient=True))
        if len(salient_rows) >= limit:
            break

    if len(salient_rows) < limit:
        for event in events:
            eid = str(event.get("event_id") or "")
            if eid in seen_ids or not display_event_label(event, canonical):
                continue
            seen_ids.add(eid)
            salient_rows.append(_timeline_row(event, canonical))
            if len(salient_rows) >= limit:
                break

    hidden_count = max(0, len(full_rows) - len(salient_rows))
    overflow: dict[str, Any] = {
        "hidden_count": hidden_count,
        "note": (
            f"{hidden_count} additional narrative event(s) omitted from the primary timeline table "
            f"(deterministic salient-first cap at {limit}); full ordering preserved in timeline_rows_full."
            if hidden_count > 0
            else ""
        ),
    }
    return salient_rows, full_rows, overflow


def build_comprehension_digest(
    narrative: dict[str, Any],
    observability: dict[str, Any] | None = None,
    *,
    log_path: Path | str | None = None,
) -> dict[str, Any]:
    """Build deterministic reviewer comprehension fields from narrative JSON."""

    summary = narrative.get("summary") if isinstance(narrative.get("summary"), dict) else {}
    lineage = narrative.get("lineage") if isinstance(narrative.get("lineage"), dict) else {}
    events = _sorted_events(narrative)
    windows = list(narrative.get("windows") or [])
    warnings = list(narrative.get("warnings") or [])
    event_by_id = _event_by_id(narrative)

    run_id = summary.get("run_id") or lineage.get("run_id") or "unknown"
    divergence_class_raw = str(summary.get("selection_oracle_divergence_class") or "not recorded")
    divergence_class_human = human_divergence_class_label(divergence_class_raw)
    event_count = summary.get("event_count", len(events))
    window_count = summary.get("window_count", len(windows))

    canonical = summary.get("canonical_parser_visible_summary")
    if not isinstance(canonical, dict):
        canonical = {}

    effective_log = log_path or lineage.get("log_path")
    log_metrics = parse_log_outcome_metrics(effective_log)
    eval_row = _evaluation_row_from_observability(observability)
    trace_summary = _trace_summary_from_observability(observability)

    outcome_context = _build_outcome_context(canonical, eval_row, log_metrics)
    selection_summary = _build_selection_summary(events, trace_summary)

    divergence_context: dict[str, Any] = {
        "taxonomy_id": divergence_class_raw,
        "human_label": divergence_class_human,
        "note": DIVERGENCE_D5_COEXISTENCE_NOTE if divergence_class_raw.startswith("D5") else "",
    }

    headline = (
        f"Derived replay summary for {run_id}: "
        f"{event_count} narrative events, {window_count} localized windows, "
        f"divergence {divergence_class_human}."
    )
    if outcome_context.get("outcome_label"):
        headline += f" Outcome: {outcome_context['outcome_label']}."

    scan_guide = [
        "This report is an explanatory visualization layer only; it does not replace parser-visible summaries or runtime authority.",
        "Start with At a glance and summary cards, then Key incidents and Figures to localize evidence along the replay log index.",
        "Divergence shading and labels localize replay-side disagreement; they are not causal proof.",
        "D-class labels are replay-local taxonomy (D0–D5); D5 indicates visibility-limited evidence, not operational severity.",
        "Sparse position and engagement panels show log-evidenced samples only when present; they are not continuous trajectories.",
        "Read Warnings and Interpretation caveats before drawing conclusions about readiness or robustness.",
    ]

    at_a_glance: list[dict[str, str]] = [
        {"label": "Run", "value": str(run_id)},
        {"label": "Divergence class", "value": divergence_class_human},
        {"label": "Taxonomy ID (parser-visible)", "value": divergence_class_raw},
        {"label": "Parser-visible outcome", "value": str(outcome_context.get("outcome_label") or "")},
    ]
    if outcome_context.get("miss_distance_m") is not None:
        at_a_glance.append(
            {
                "label": "Minimum miss distance (m)",
                "value": f"{outcome_context['miss_distance_m']}",
            }
        )
    if outcome_context.get("hit_threshold_m") is not None:
        at_a_glance.append(
            {
                "label": "Hit threshold (m)",
                "value": f"{outcome_context['hit_threshold_m']}",
            }
        )
    if outcome_context.get("margin_to_threshold_m") is not None:
        at_a_glance.append(
            {
                "label": "Margin to parser-visible threshold (m)",
                "value": f"{outcome_context['margin_to_threshold_m']}",
            }
        )
    if outcome_context.get("near_threshold_note"):
        at_a_glance.append(
            {
                "label": "Threshold proximity",
                "value": str(outcome_context["near_threshold_note"]),
            }
        )
    if selection_summary.get("mismatch_count"):
        at_a_glance.append(
            {
                "label": "Selection/oracle mismatches",
                "value": (
                    f"{selection_summary['mismatch_count']} "
                    f"(first block {selection_summary.get('first_mismatch_block', '—')})"
                ),
            }
        )
    at_a_glance.extend(
        [
            {"label": "Narrative events", "value": str(event_count)},
            {"label": "Localized windows", "value": str(window_count)},
            {"label": "Warnings", "value": str(len(warnings))},
        ]
    )

    incident_groups, selection_detail_rows = _build_incident_groups(events, canonical, selection_summary)
    timeline_rows_salient, timeline_rows_full, timeline_overflow = _build_timeline_salient(events, canonical)

    key_windows: list[dict[str, str]] = []
    for window in windows:
        window_type = str(window.get("window_type") or "")
        start_id = window.get("start_event_id")
        start_label = ""
        if start_id and str(start_id) in event_by_id:
            start_label = display_event_label(event_by_id[str(start_id)], canonical)
        key_windows.append(
            {
                "window_type": window_type,
                "window_label": WINDOW_TYPE_LABELS.get(window_type, window_type.replace("_", " ")),
                "start_line_index": _format_line_index(window.get("start_line_index")),
                "end_line_index": _format_line_index(window.get("end_line_index")),
                "start_event_label": start_label,
            }
        )

    return {
        "at_a_glance": at_a_glance,
        "divergence_context": divergence_context,
        "figure_display_order": list(FIGURE_DISPLAY_ORDER),
        "figure_look_for": dict(FIGURE_LOOK_FOR),
        "figure_reviewer_titles": dict(FIGURE_REVIEWER_TITLES),
        "headline": headline,
        "incident_groups": incident_groups,
        "key_windows": key_windows,
        "outcome_context": outcome_context,
        "scan_guide": scan_guide,
        "selection_detail_rows": selection_detail_rows,
        "selection_summary": selection_summary,
        "timeline_overflow": timeline_overflow,
        "timeline_rows": timeline_rows_salient,
        "timeline_rows_full": timeline_rows_full,
        "timeline_rows_salient": timeline_rows_salient,
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
        or str(e.get("category") or "") in ("divergence", "provenance_warning", "outcome")
    ]
    return sorted(candidates, key=priority)[:limit]
