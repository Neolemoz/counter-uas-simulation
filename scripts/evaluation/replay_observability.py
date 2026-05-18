#!/usr/bin/env python3
"""Replay observability reports for evidence-side review workflows.

All outputs from this module are derived evaluation artifacts. They preserve
raw replay lineage and do not replace parser-visible summaries, runtime topics,
or tactical authority surfaces.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import re
import shlex
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

_SCRIPTS_DIR = Path(__file__).resolve().parents[1]
_EVAL_DIR = Path(__file__).resolve().parent
for p in (_SCRIPTS_DIR, _EVAL_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from classify_selection_oracle_divergence import classify_selection_oracle_divergence  # noqa: E402
from evaluation_row import evaluation_row  # noqa: E402
from pair_mc_seed_outcomes import seed_from_meta  # noqa: E402
from realism_metrics import realism_metrics_summary  # noqa: E402
from selection_audit import extract_selection_blocks  # noqa: E402
from selection_audit import observer_visibility_summary  # noqa: E402
from selection_audit import selection_audit_summary  # noqa: E402


SCHEMA_VERSION = "replay_observability_v1"
NON_AUTHORITATIVE_NOTICE = (
    "Derived evaluation artifact only. Does not replace parser-visible summaries, "
    "runtime topics, tactical authority, lifecycle semantics, or replay contracts."
)
EVIDENCE_ROLES = {
    "raw_runtime_evidence": "Original log/meta/topic-derived evidence captured from a run.",
    "canonical_parser_visible_summary": "Existing parser-visible success/miss/intercept-time surface.",
    "derived_evaluation_artifact": "Additive, replay-safe evaluation-side derivation.",
    "explanatory_visualization_layer": "Reviewer convenience projection with preserved lineage.",
}
CANONICAL_SUMMARY_FIELDS = (
    "run_id",
    "success",
    "hit",
    "miss_distance_m",
    "intercept_time_s",
    "time_margin_s",
    "layer_at_hit",
    "notes",
)
LIFECYCLE_EVENT_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("realism_fragmented_gap_start", re.compile(r"\[REALISM_EVENT\]\s+fragmented_gap_start\b")),
    ("realism_fragmented_gap_end", re.compile(r"\[REALISM_EVENT\]\s+fragmented_gap_end\b")),
    ("realism_delayed_detection", re.compile(r"\[REALISM_EVENT\]\s+delayed_detection\b")),
    ("realism_stale_detection", re.compile(r"\[REALISM_EVENT\]\s+stale_detection\b")),
    ("realism_dropout_burst_start", re.compile(r"\[REALISM_EVENT\]\s+dropout_burst_start\b")),
    ("realism_ghost_detection", re.compile(r"\[REALISM_EVENT\]\s+ghost_detection\b")),
    ("lifecycle_observer_summary", re.compile(r"\[LIFECYCLE_OBSERVER\]\s+event=summary\b")),
    ("lifecycle_observer_selected_id", re.compile(r"\[LIFECYCLE_OBSERVER\]\s+event=selected_id\b")),
    ("track_continuity_gap", re.compile(r"\[TRACK_CONTINUITY\]\s+event=track_gap\b")),
    ("track_continuity_change", re.compile(r"\[TRACK_CONTINUITY\]\s+event=track_id_change\b")),
    ("selection_proxy_window", re.compile(r"\[SELECTION_PROXY\]\s+event=track_persistence_window\b")),
    ("track_persistence_boundary", re.compile(r"\[TRACK_PERSISTENCE\]\s+event=track_persistence_boundary\b")),
    ("candidate_spawn", re.compile(r"Candidate detected:")),
    ("candidate_discard", re.compile(r"Candidate discarded:")),
    ("duplicate_track_merge", re.compile(r"Merging track\s+\d+\s+and track\s+\d+")),
    ("track_coast", re.compile(r"Track\s+\d+\s+NOT updated \(missed_frames=\d+\)")),
    ("track_recovery", re.compile(r"Track\s+\d+\s+updated \(valid match\)")),
)


def _read_json(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {"_read_error": "unreadable_json"}
    return data if isinstance(data, dict) else {"_read_error": "json_not_object"}


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")


def _maybe_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _safe_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() == "true"


def _sidecar_meta_path(log_path: Path) -> Path:
    return log_path.with_suffix(".meta.json")


def _sidecar_eval_path(log_path: Path) -> Path:
    return log_path.with_suffix(".evaluation_row.json")


def _log_path_from_row(row: dict[str, Any]) -> Path | None:
    raw = str(row.get("log_path") or row.get("log") or "").strip()
    return Path(raw) if raw else None


def _parse_seed_from_text(text: str) -> int | None:
    for pattern in (r"\bnoise_seed(?:_mc)?(?::=|:|=)(\d+)\b", r"\bseed(?::=|:|=)(\d+)\b"):
        m = re.search(pattern, text)
        if m:
            return int(m.group(1))
    return None


def _seed_from_row_or_meta(row: dict[str, Any], meta: dict[str, Any], log_path: Path | None) -> tuple[int | None, str]:
    for key in ("noise_seed_mc", "seed"):
        raw = row.get(key)
        if raw not in (None, ""):
            try:
                return int(raw), f"row.{key}"
            except (TypeError, ValueError):
                pass
    if log_path is not None:
        sid = seed_from_meta(str(log_path))
        if sid is not None:
            return sid, "meta.notes"
    notes_seed = _parse_seed_from_text(str(meta.get("notes") or ""))
    if notes_seed is not None:
        return notes_seed, "meta.notes"
    cmd_seed = _parse_seed_from_text(" ".join(str(x) for x in meta.get("cmd", [])))
    if cmd_seed is not None:
        return cmd_seed, "meta.cmd"
    return None, "missing"


def _parse_launch_args(launch_args: str) -> dict[str, str]:
    out: dict[str, str] = {}
    if not launch_args:
        return out
    try:
        tokens = shlex.split(launch_args)
    except ValueError:
        tokens = launch_args.split()
    for token in tokens:
        if ":=" not in token:
            continue
        key, value = token.split(":=", 1)
        if key:
            out[key] = value
    return out


def _launch_args_from_meta(meta: dict[str, Any]) -> str:
    cmd = meta.get("cmd")
    if isinstance(cmd, list):
        joined = " ".join(str(x) for x in cmd)
    else:
        joined = str(cmd or "")
    if "ros2 launch" not in joined:
        return ""
    # Preserve the original text; downstream consumers can inspect parsed key/value pairs.
    return joined


def _profile_label(profile_id: str, notes: str) -> str:
    blob = f"{profile_id} {notes}".lower()
    if "baseline" in blob:
        return "reachability_baseline"
    if "phase3" in blob or "bounded" in blob:
        return "bounded_fragmented_point"
    if "phase1" in blob or "sentinel" in blob:
        return "instability_sentinel"
    return "unclassified_profile"


def _governance_block() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "notice": NON_AUTHORITATIVE_NOTICE,
        "artifact_roles": EVIDENCE_ROLES,
        "constraints": [
            "evaluation-side only",
            "additive-only",
            "parser-safe",
            "replay-safe",
            "non-authoritative",
            "preserve original replay lineage",
        ],
        "anti_claims": [
            "not operational readiness evidence",
            "not hardware readiness evidence",
            "not a tactical authority surface",
            "not a lifecycle semantic replacement",
            "not a parser contract",
        ],
    }


def build_evidence_bundle(
    log_path: Path,
    *,
    meta_path: Path | None = None,
    evaluation_row_path: Path | None = None,
) -> dict[str, Any]:
    """Build a provenance-preserving replay evidence bundle for one run."""

    log_path = Path(log_path)
    meta_path = meta_path if meta_path is not None else _sidecar_meta_path(log_path)
    evaluation_row_path = evaluation_row_path if evaluation_row_path is not None else _sidecar_eval_path(log_path)

    meta = _read_json(meta_path)
    if evaluation_row_path.is_file():
        row = _read_json(evaluation_row_path)
    elif log_path.is_file():
        row = evaluation_row(log_path, meta_path=meta_path if meta_path.is_file() else None)
    else:
        row = {}

    seed, seed_source = _seed_from_row_or_meta(row, meta, log_path)
    canonical = {k: row.get(k) for k in CANONICAL_SUMMARY_FIELDS if k in row}
    additive_keys = sorted(k for k in row if k not in canonical and k not in {"git_commit", "git_dirty", "notes_meta"})
    warnings: list[str] = []
    if not log_path.is_file():
        warnings.append("raw log is missing")
    if not meta_path.is_file():
        warnings.append("sidecar meta is missing")
    if seed is None:
        warnings.append("seed lineage is missing or could not be parsed")
    if meta.get("_read_error"):
        warnings.append(f"meta read issue: {meta['_read_error']}")
    if row.get("_read_error"):
        warnings.append(f"evaluation row read issue: {row['_read_error']}")

    return {
        "artifact_type": "replay_evidence_bundle",
        "governance": _governance_block(),
        "lineage": {
            "run_id": row.get("run_id") or meta.get("run_id") or log_path.stem,
            "log_path": str(log_path),
            "meta_path": str(meta_path),
            "evaluation_row_path": str(evaluation_row_path) if evaluation_row_path.is_file() else None,
            "git_commit": row.get("git_commit") or meta.get("git_commit"),
            "git_dirty": row.get("git_dirty") if row.get("git_dirty") is not None else meta.get("git_dirty"),
            "cohort": meta.get("cohort"),
            "seed": seed,
            "seed_source": seed_source,
            "created_utc": meta.get("created_utc"),
            "launch_command": meta.get("cmd"),
            "notes": meta.get("notes") or row.get("notes_meta"),
        },
        "evidence_layers": {
            "raw_runtime_evidence": {
                "log": str(log_path),
                "meta": str(meta_path) if meta_path.is_file() else None,
            },
            "canonical_parser_visible_summary": canonical,
            "derived_evaluation_artifacts": {
                "evaluation_row": row,
                "additive_field_names": additive_keys,
                "selection_oracle_divergence_class": row.get("selection_oracle_divergence_class"),
                "ambiguity_failure_class": row.get("ambiguity_failure_class"),
                "realism_failure_class": row.get("realism_failure_class"),
            },
            "explanatory_visualization_inputs": [],
        },
        "warnings": warnings,
    }


def build_divergence_trace(log_path: Path, *, meta_path: Path | None = None) -> dict[str, Any]:
    """Build an inspectable selection/oracle divergence trace for one replay log."""

    text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    row = evaluation_row(Path(log_path), meta_path=meta_path if meta_path and meta_path.is_file() else None)
    blocks = extract_selection_blocks(text)
    trace_blocks: list[dict[str, Any]] = []
    for block in blocks:
        trace_blocks.append(
            {
                "block_index": block.get("block_index"),
                "line_index": block.get("line_index"),
                "selected": block.get("selected"),
                "oracle_ids": block.get("oracle_ids"),
                "oracle_tti_min": block.get("oracle_tti_min"),
                "oracle_match": block.get("oracle_match"),
                "after_fragmented_gap": block.get("after_fragmented_gap"),
                "evidence_role": "derived_evaluation_artifact",
            }
        )
    mismatch_blocks = [b for b in trace_blocks if b.get("oracle_match") is False]
    return {
        "artifact_type": "divergence_trace",
        "governance": _governance_block(),
        "lineage": {
            "log_path": str(log_path),
            "meta_path": str(meta_path) if meta_path else None,
            "run_id": row.get("run_id") or Path(log_path).stem,
        },
        "summary": {
            "selection_oracle_divergence_class": row.get("selection_oracle_divergence_class")
            or classify_selection_oracle_divergence(row),
            "n_selection_blocks": row.get("n_selection_blocks"),
            "selection_oracle_match_rate": row.get("selection_oracle_match_rate"),
            "first_selection_mismatch_block": row.get("first_selection_mismatch_block"),
            "last_selection_mismatch_block": row.get("last_selection_mismatch_block"),
            "selection_mismatch_count": row.get("selection_mismatch_count"),
            "mismatch_after_fragmented_gap_count": row.get("mismatch_after_fragmented_gap_count"),
        },
        "blocks": trace_blocks,
        "mismatch_blocks": mismatch_blocks,
        "interpretation_caveats": [
            "Block indices and line indices localize replay-side evidence only.",
            "Divergence classes are additive evaluation taxonomy labels, not authority semantics.",
            "Oracle comparison does not replace runtime selected-id authority surfaces.",
        ],
    }


def build_lifecycle_timeline(log_path: Path) -> dict[str, Any]:
    """Build an explanatory lifecycle/churn timeline with raw line provenance."""

    text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    events: list[dict[str, Any]] = []
    for idx, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        for event_type, pattern in LIFECYCLE_EVENT_PATTERNS:
            if pattern.search(line):
                events.append(
                    {
                        "line_index": idx,
                        "event_type": event_type,
                        "raw_line": line,
                        "evidence_role": "explanatory_visualization_layer",
                    }
                )
    counts = Counter(str(e["event_type"]) for e in events)
    return {
        "artifact_type": "lifecycle_churn_timeline",
        "governance": _governance_block(),
        "lineage": {
            "log_path": str(log_path),
            "run_id": Path(log_path).stem,
        },
        "summary": {
            "event_counts": dict(sorted(counts.items())),
            "realism_metrics": realism_metrics_summary(text),
            "observer_visibility": observer_visibility_summary(text),
            "selection_audit": selection_audit_summary(text),
        },
        "events": events,
        "interpretation_caveats": [
            "Timeline events are explanatory overlays over original log lines.",
            "Lifecycle/churn counters are not proof of tracker robustness.",
            "Original event names and raw line provenance are preserved.",
        ],
    }


def build_single_run_report(log_path: Path, *, meta_path: Path | None = None) -> dict[str, Any]:
    """Bundle R1/R2 single-run evidence, divergence, and lifecycle timeline."""

    meta_path = meta_path if meta_path is not None else _sidecar_meta_path(Path(log_path))
    return {
        "artifact_type": "single_run_replay_observability_report",
        "governance": _governance_block(),
        "bundle": build_evidence_bundle(Path(log_path), meta_path=meta_path),
        "divergence_trace": build_divergence_trace(Path(log_path), meta_path=meta_path),
        "lifecycle_timeline": build_lifecycle_timeline(Path(log_path)),
    }


def _load_csv_rows(path: Path) -> list[dict[str, Any]]:
    with Path(path).open(encoding="utf-8", newline="") as f:
        return [dict(r) for r in csv.DictReader(f)]


def _row_meta(row: dict[str, Any]) -> tuple[Path | None, dict[str, Any]]:
    log_path = _log_path_from_row(row)
    meta_path = _sidecar_meta_path(log_path) if log_path else None
    return log_path, _read_json(meta_path)


def _row_success(row: dict[str, Any]) -> bool:
    return _safe_bool(row.get("success"))


def _paired_bucket(base: dict[str, Any], cand: dict[str, Any], *, miss_delta: float, tint_delta: float) -> str:
    bs = _row_success(base)
    cs = _row_success(cand)
    miss_b = _maybe_float(base.get("miss_distance_m")) or 0.0
    miss_c = _maybe_float(cand.get("miss_distance_m")) or 0.0
    tint_b = _maybe_float(base.get("intercept_time_s")) or 0.0
    tint_c = _maybe_float(cand.get("intercept_time_s")) or 0.0
    if bs and not cs:
        return "G1_base_ok_cand_fail"
    if cs and not bs:
        return "G2_cand_ok_base_fail"
    if not bs and not cs:
        return "G4_both_fail"
    if miss_c > miss_b + miss_delta or tint_c > tint_b + tint_delta:
        return "G3_both_ok_cand_worse"
    return "G0_both_ok_cand_not_worse"


def _map_rows_by_seed(rows: list[dict[str, Any]]) -> tuple[dict[int, dict[str, Any]], list[str]]:
    out: dict[int, dict[str, Any]] = {}
    warnings: list[str] = []
    for row in rows:
        log_path, meta = _row_meta(row)
        seed, seed_source = _seed_from_row_or_meta(row, meta, log_path)
        if seed is None:
            warnings.append(f"missing seed for row {row.get('run_id') or row.get('log_path') or '<unknown>'}")
            continue
        row = dict(row)
        row["_seed_source"] = seed_source
        row["_meta"] = meta
        out[seed] = row
    return out, warnings


def _evaluation_summary_for_row(row: dict[str, Any]) -> dict[str, Any]:
    log_path = _log_path_from_row(row)
    if log_path is None or not log_path.is_file():
        return {}
    meta_path = _sidecar_meta_path(log_path)
    try:
        ev = evaluation_row(log_path, meta_path=meta_path if meta_path.is_file() else None)
    except OSError:
        return {}
    return {
        "selection_oracle_divergence_class": ev.get("selection_oracle_divergence_class"),
        "first_selection_mismatch_block": ev.get("first_selection_mismatch_block"),
        "selection_mismatch_count": ev.get("selection_mismatch_count"),
        "mismatch_after_fragmented_gap_count": ev.get("mismatch_after_fragmented_gap_count"),
        "lifecycle_thrash_score": ev.get("lifecycle_thrash_score"),
        "candidate_churn_pressure_count": ev.get("candidate_churn_pressure_count"),
        "track_coast_count": ev.get("track_coast_count"),
        "track_recovery_count": ev.get("track_recovery_count"),
    }


def build_matched_seed_report(
    baseline_csv: Path,
    candidate_csv: Path,
    *,
    miss_delta: float = 0.02,
    tint_delta: float = 0.05,
) -> dict[str, Any]:
    """Build a paired-seed comparison report over existing MC CSVs."""

    base_rows, base_warnings = _map_rows_by_seed(_load_csv_rows(baseline_csv))
    cand_rows, cand_warnings = _map_rows_by_seed(_load_csv_rows(candidate_csv))
    seeds = sorted(set(base_rows) & set(cand_rows))
    paired: list[dict[str, Any]] = []
    bucket_counts: Counter[str] = Counter()
    cohort_pairs: Counter[tuple[str, str]] = Counter()

    for seed in seeds:
        b = base_rows[seed]
        c = cand_rows[seed]
        bucket = _paired_bucket(b, c, miss_delta=miss_delta, tint_delta=tint_delta)
        bucket_counts[bucket] += 1
        b_meta = b.get("_meta") if isinstance(b.get("_meta"), dict) else {}
        c_meta = c.get("_meta") if isinstance(c.get("_meta"), dict) else {}
        cohort_pairs[(str(b_meta.get("cohort") or ""), str(c_meta.get("cohort") or ""))] += 1
        paired.append(
            {
                "seed": seed,
                "bucket": bucket,
                "base": {
                    "success": _row_success(b),
                    "miss_distance_m": _maybe_float(b.get("miss_distance_m")),
                    "intercept_time_s": _maybe_float(b.get("intercept_time_s")),
                    "log_path": b.get("log_path"),
                    "seed_source": b.get("_seed_source"),
                    "cohort": b_meta.get("cohort"),
                    "evaluation_summary": _evaluation_summary_for_row(b),
                },
                "candidate": {
                    "success": _row_success(c),
                    "miss_distance_m": _maybe_float(c.get("miss_distance_m")),
                    "intercept_time_s": _maybe_float(c.get("intercept_time_s")),
                    "log_path": c.get("log_path"),
                    "seed_source": c.get("_seed_source"),
                    "cohort": c_meta.get("cohort"),
                    "evaluation_summary": _evaluation_summary_for_row(c),
                },
            }
        )

    warnings = base_warnings + cand_warnings
    if set(base_rows) - set(cand_rows):
        warnings.append(f"baseline-only seeds: {sorted(set(base_rows) - set(cand_rows))[:20]}")
    if set(cand_rows) - set(base_rows):
        warnings.append(f"candidate-only seeds: {sorted(set(cand_rows) - set(base_rows))[:20]}")
    if len(cohort_pairs) > 1:
        warnings.append("multiple cohort pairings detected; review comparability before interpreting deltas")

    return {
        "artifact_type": "matched_seed_comparison_report",
        "governance": _governance_block(),
        "inputs": {
            "baseline_csv": str(baseline_csv),
            "candidate_csv": str(candidate_csv),
            "miss_delta": miss_delta,
            "tint_delta": tint_delta,
        },
        "summary": {
            "paired_seed_count": len(seeds),
            "bucket_counts": dict(sorted(bucket_counts.items())),
            "cohort_pairs": {f"{a} -> {b}": n for (a, b), n in sorted(cohort_pairs.items())},
        },
        "paired_seeds": paired,
        "warnings": warnings,
        "interpretation_caveats": [
            "Matched seeds support descriptive comparison, not statistical superiority claims.",
            "Seed, cohort, geometry, and launch assumptions must remain visible for review.",
            "Notes-derived seed lineage is lower confidence than structured row seed fields.",
        ],
    }


def build_topology_index(
    profiles_csv: Path,
    *,
    summary_csv: Path | None = None,
) -> dict[str, Any]:
    """Index profile/topology/timing metadata without redefining topology semantics."""

    summary_by_profile: dict[str, dict[str, Any]] = {}
    if summary_csv and summary_csv.is_file():
        for row in _load_csv_rows(summary_csv):
            pid = str(row.get("profile_id") or "").strip()
            if pid:
                summary_by_profile[pid] = row

    records: list[dict[str, Any]] = []
    for row in _load_csv_rows(profiles_csv):
        profile_id = str(row.get("profile_id") or "").strip()
        launch_args = str(row.get("launch_args") or "")
        parsed = _parse_launch_args(launch_args)
        geometry = {
            "target_start_x_m": parsed.get("target_start_x_m"),
            "target_start_y_m": parsed.get("target_start_y_m"),
            "target_start_z_m": parsed.get("target_start_z_m"),
        }
        fragmentation = {
            "enabled": parsed.get("fragmentation_staggered_enabled", "false"),
            "cycle_ticks": parsed.get("fragmentation_stagger_cycle_ticks"),
            "phase_ticks": parsed.get("fragmentation_stagger_phase_ticks"),
            "gap_ticks": parsed.get("fragmentation_stagger_gap_ticks"),
        }
        records.append(
            {
                "profile_id": profile_id,
                "scenario": row.get("scenario"),
                "notes": row.get("notes"),
                "profile_label": _profile_label(profile_id, str(row.get("notes") or "")),
                "launch_args": launch_args,
                "parsed_launch_args": parsed,
                "geometry": geometry,
                "fragmentation": fragmentation,
                "observer_enabled": parsed.get("enable_lifecycle_observer"),
                "noise_seed": parsed.get("noise_seed"),
                "summary": summary_by_profile.get(profile_id, {}),
                "evidence_role": "derived_evaluation_artifact",
            }
        )

    timing_groups: dict[str, list[str]] = defaultdict(list)
    geometry_groups: dict[str, list[str]] = defaultdict(list)
    for record in records:
        frag = record["fragmentation"]
        timing_key = (
            f"enabled={frag.get('enabled')} cycle={frag.get('cycle_ticks')} "
            f"gap={frag.get('gap_ticks')} phase={frag.get('phase_ticks')}"
        )
        geom = record["geometry"]
        geometry_key = f"x={geom.get('target_start_x_m')} y={geom.get('target_start_y_m')} z={geom.get('target_start_z_m')}"
        timing_groups[timing_key].append(str(record["profile_id"]))
        geometry_groups[geometry_key].append(str(record["profile_id"]))

    return {
        "artifact_type": "topology_timing_analytics_index",
        "governance": _governance_block(),
        "inputs": {
            "profiles_csv": str(profiles_csv),
            "summary_csv": str(summary_csv) if summary_csv else None,
        },
        "summary": {
            "profile_count": len(records),
            "timing_group_count": len(timing_groups),
            "geometry_group_count": len(geometry_groups),
            "timing_groups": dict(sorted(timing_groups.items())),
            "geometry_groups": dict(sorted(geometry_groups.items())),
        },
        "records": records,
        "interpretation_caveats": [
            "Profile IDs and launch arguments remain the source lineage for topology/timing labels.",
            "Timing groups are descriptive evaluation groupings, not new runtime timing semantics.",
            "Transferability findings must remain linked to geometry/profile evidence.",
        ],
    }


def lint_governance_artifact(payload: dict[str, Any]) -> dict[str, Any]:
    """Check a derived artifact for minimum governance/provenance labels."""

    issues: list[str] = []
    governance = payload.get("governance")
    if not isinstance(governance, dict):
        issues.append("missing governance block")
    else:
        if governance.get("schema_version") != SCHEMA_VERSION:
            issues.append("missing or unexpected governance schema_version")
        notice = str(governance.get("notice") or "")
        if "Derived evaluation artifact" not in notice:
            issues.append("governance notice must identify derived evaluation artifact status")
        constraints = set(governance.get("constraints") or [])
        for required in ("evaluation-side only", "additive-only", "parser-safe", "replay-safe", "non-authoritative"):
            if required not in constraints:
                issues.append(f"missing governance constraint: {required}")
    if not payload.get("artifact_type"):
        issues.append("missing artifact_type")
    text = json.dumps(payload, sort_keys=True, default=str)
    if "operational readiness" in text and "not operational readiness evidence" not in text:
        issues.append("operational-readiness language lacks explicit anti-claim")
    return {
        "artifact_type": "governance_lint_result",
        "governance": _governance_block(),
        "ok": not issues,
        "issues": issues,
    }


def render_static_markdown(payload: dict[str, Any]) -> str:
    """Render a compact reviewer-facing markdown report from a derived artifact."""

    artifact_type = str(payload.get("artifact_type") or "replay_observability_artifact")
    lines = [
        f"# {artifact_type.replace('_', ' ').title()}",
        "",
        f"**Governance:** {NON_AUTHORITATIVE_NOTICE}",
        "",
    ]
    if "lineage" in payload:
        lines.extend(["## Lineage", "", "```json", json.dumps(payload["lineage"], indent=2, sort_keys=True, default=str), "```", ""])
    if "summary" in payload:
        lines.extend(["## Summary", "", "```json", json.dumps(payload["summary"], indent=2, sort_keys=True, default=str), "```", ""])
    if "warnings" in payload and payload["warnings"]:
        lines.extend(["## Warnings", ""])
        lines.extend(f"- {w}" for w in payload["warnings"])
        lines.append("")
    if artifact_type == "single_run_replay_observability_report":
        bundle = payload.get("bundle", {})
        trace = payload.get("divergence_trace", {})
        timeline = payload.get("lifecycle_timeline", {})
        lines.extend(
            [
                "## Single Run Evidence",
                "",
                "```json",
                json.dumps(bundle.get("lineage", {}), indent=2, sort_keys=True, default=str),
                "```",
                "",
                "## Divergence",
                "",
                "```json",
                json.dumps(trace.get("summary", {}), indent=2, sort_keys=True, default=str),
                "```",
                "",
                "## Lifecycle / Churn",
                "",
                "```json",
                json.dumps(timeline.get("summary", {}).get("event_counts", {}), indent=2, sort_keys=True, default=str),
                "```",
                "",
            ]
        )
    lines.extend(["## Caveats", ""])
    caveats = payload.get("interpretation_caveats") or payload.get("governance", {}).get("anti_claims") or []
    lines.extend(f"- {c}" for c in caveats)
    lines.append("")
    return "\n".join(lines)


def render_static_html(markdown_text: str) -> str:
    """Render markdown-ish text into a minimal static HTML reviewer artifact."""

    return (
        "<!doctype html><html><head><meta charset=\"utf-8\"><title>Replay Observability Report</title>"
        "<style>body{font-family:sans-serif;max-width:960px;margin:2rem auto;line-height:1.45;}"
        "pre{background:#f6f8fa;padding:1rem;overflow:auto;}code{font-family:monospace;}</style>"
        "</head><body><pre>"
        + html.escape(markdown_text)
        + "</pre></body></html>\n"
    )


def _html_cell(value: Any) -> str:
    return html.escape("" if value is None else str(value))


def _markdown_cell(value: Any) -> str:
    text = "" if value is None else str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def _markdown_table(headers: list[str], rows: list[list[Any]]) -> list[str]:
    if not rows:
        return ["_No rows._", ""]
    out = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        out.append("| " + " | ".join(_markdown_cell(v) for v in row) + " |")
    out.append("")
    return out


def _html_table(headers: list[str], rows: list[list[Any]]) -> str:
    if not rows:
        return "<p><em>No rows.</em></p>"
    head = "".join(f"<th>{_html_cell(h)}</th>" for h in headers)
    body_rows = []
    for row in rows:
        body_rows.append("<tr>" + "".join(f"<td>{_html_cell(v)}</td>" for v in row) + "</tr>")
    return "<table><thead><tr>" + head + "</tr></thead><tbody>" + "".join(body_rows) + "</tbody></table>"


def _dashboard_inputs(
    *,
    single_run: dict[str, Any] | None = None,
    paired_comparison: dict[str, Any] | None = None,
    topology_index: dict[str, Any] | None = None,
    governance_lint: dict[str, Any] | None = None,
    input_paths: dict[str, str] | None = None,
) -> dict[str, Any]:
    return {
        "single_run": single_run or {},
        "paired_comparison": paired_comparison or {},
        "topology_index": topology_index or {},
        "governance_lint": governance_lint or {},
        "input_paths": input_paths or {},
    }


def _dashboard_warnings(data: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    single = data["single_run"]
    bundle = single.get("bundle", {}) if isinstance(single, dict) else {}
    for source in (
        bundle,
        data["paired_comparison"],
        data["topology_index"],
        data["governance_lint"],
    ):
        if not isinstance(source, dict):
            continue
        for warning in source.get("warnings") or []:
            warnings.append(str(warning))
        for issue in source.get("issues") or []:
            warnings.append(f"governance issue: {issue}")
    return sorted(dict.fromkeys(warnings))


def _single_run_rows(single: dict[str, Any]) -> tuple[list[list[Any]], list[list[Any]], list[list[Any]]]:
    bundle = single.get("bundle", {}) if isinstance(single, dict) else {}
    lineage = bundle.get("lineage", {}) if isinstance(bundle, dict) else {}
    trace = single.get("divergence_trace", {}) if isinstance(single, dict) else {}
    timeline = single.get("lifecycle_timeline", {}) if isinstance(single, dict) else {}
    trace_summary = trace.get("summary", {}) if isinstance(trace, dict) else {}
    event_counts = (
        timeline.get("summary", {}).get("event_counts", {})
        if isinstance(timeline, dict)
        else {}
    )
    card = [
        ["run_id", lineage.get("run_id")],
        ["seed", lineage.get("seed")],
        ["seed_source", lineage.get("seed_source")],
        ["cohort", lineage.get("cohort")],
        ["git_commit", lineage.get("git_commit")],
        ["git_dirty", lineage.get("git_dirty")],
        ["log_path", lineage.get("log_path")],
        ["meta_path", lineage.get("meta_path")],
        ["divergence_class", trace_summary.get("selection_oracle_divergence_class")],
        ["first_mismatch_block", trace_summary.get("first_selection_mismatch_block")],
        ["selection_mismatch_count", trace_summary.get("selection_mismatch_count")],
    ]
    divergence_rows = []
    for block in trace.get("blocks", []) if isinstance(trace, dict) else []:
        divergence_rows.append(
            [
                block.get("block_index"),
                block.get("line_index"),
                block.get("selected"),
                ",".join(str(v) for v in block.get("oracle_ids", [])),
                block.get("oracle_match"),
                block.get("after_fragmented_gap"),
            ]
        )
    lifecycle_rows = []
    for name in sorted(event_counts):
        lifecycle_rows.append([name, event_counts[name]])
    return card, divergence_rows, lifecycle_rows


def _paired_rows(paired: dict[str, Any]) -> tuple[list[list[Any]], list[list[Any]]]:
    summary = paired.get("summary", {}) if isinstance(paired, dict) else {}
    bucket_rows = [[k, v] for k, v in sorted((summary.get("bucket_counts") or {}).items())]
    seed_rows = []
    for row in paired.get("paired_seeds", []) if isinstance(paired, dict) else []:
        base = row.get("base", {})
        cand = row.get("candidate", {})
        seed_rows.append(
            [
                row.get("seed"),
                row.get("bucket"),
                base.get("success"),
                cand.get("success"),
                base.get("seed_source"),
                cand.get("seed_source"),
                base.get("cohort"),
                cand.get("cohort"),
                base.get("log_path"),
                cand.get("log_path"),
            ]
        )
    seed_rows.sort(key=lambda r: (str(r[0]), str(r[1])))
    return bucket_rows, seed_rows


def _topology_rows(topology: dict[str, Any]) -> tuple[list[list[Any]], list[list[Any]]]:
    summary = topology.get("summary", {}) if isinstance(topology, dict) else {}
    timing_rows = [[k, ", ".join(v)] for k, v in sorted((summary.get("timing_groups") or {}).items())]
    profile_rows = []
    for record in topology.get("records", []) if isinstance(topology, dict) else []:
        geometry = record.get("geometry", {})
        fragmentation = record.get("fragmentation", {})
        profile_rows.append(
            [
                record.get("profile_id"),
                record.get("scenario"),
                record.get("profile_label"),
                geometry.get("target_start_x_m"),
                geometry.get("target_start_y_m"),
                geometry.get("target_start_z_m"),
                fragmentation.get("cycle_ticks"),
                fragmentation.get("gap_ticks"),
                fragmentation.get("phase_ticks"),
                record.get("observer_enabled"),
                record.get("noise_seed"),
                record.get("launch_args"),
                record.get("notes"),
            ]
        )
    profile_rows.sort(key=lambda r: (str(r[0]), str(r[1])))
    return timing_rows, profile_rows


def render_dashboard_markdown(
    *,
    single_run: dict[str, Any] | None = None,
    paired_comparison: dict[str, Any] | None = None,
    topology_index: dict[str, Any] | None = None,
    governance_lint: dict[str, Any] | None = None,
    input_paths: dict[str, str] | None = None,
) -> str:
    """Render a static, non-authoritative reviewer dashboard in markdown."""

    data = _dashboard_inputs(
        single_run=single_run,
        paired_comparison=paired_comparison,
        topology_index=topology_index,
        governance_lint=governance_lint,
        input_paths=input_paths,
    )
    warnings = _dashboard_warnings(data)
    single_card, divergence_rows, lifecycle_rows = _single_run_rows(data["single_run"])
    bucket_rows, seed_rows = _paired_rows(data["paired_comparison"])
    timing_rows, profile_rows = _topology_rows(data["topology_index"])
    lines = [
        "# Replay Reviewer Static Dashboard",
        "",
        f"**Governance:** {NON_AUTHORITATIVE_NOTICE}",
        "",
        "This dashboard is an explanatory visualization layer. It is not a parser contract, "
        "runtime authority surface, lifecycle semantic replacement, or operational readiness claim.",
        "",
        "## Evidence Layer Labels",
        "",
        "- Raw runtime evidence: original logs and sidecar metadata.",
        "- Canonical parser-visible summary: existing success/miss/intercept-time fields.",
        "- Derived evaluation artifact: additive replay observability JSON.",
        "- Explanatory visualization layer: this static dashboard.",
        "",
        "## Warning Panel",
        "",
    ]
    lines.extend(f"- {w}" for w in warnings or ["No warnings reported by supplied artifacts."])
    lines.append("")
    lines.extend(["## Provenance / Evidence Map", ""])
    input_rows = [[k, v] for k, v in sorted(data["input_paths"].items())]
    lines.extend(_markdown_table(["artifact", "path"], input_rows))
    lines.extend(["## Single-Run Replay Card", ""])
    lines.extend(_markdown_table(["field", "value"], single_card))
    lines.extend(["## Divergence Timeline Table", ""])
    lines.append("Temporal context only: fragmented-gap adjacency is not causal proof.")
    lines.append("")
    lines.extend(_markdown_table(["block", "line", "selected", "oracle_ids", "match", "after_fragmented_gap"], divergence_rows))
    lines.extend(["## Lifecycle / Churn Overlay", ""])
    lines.append("Explanatory overlay only: lifecycle/churn counters are not tracker truth.")
    lines.append("")
    lines.extend(_markdown_table(["event_type", "count"], lifecycle_rows))
    lines.extend(["## Matched-Seed Comparison", ""])
    lines.append("Descriptive comparison only: no statistical superiority claim is made.")
    lines.append("")
    lines.extend(_markdown_table(["bucket", "count"], bucket_rows))
    lines.extend(
        _markdown_table(
            [
                "seed",
                "bucket",
                "base_success",
                "candidate_success",
                "base_seed_source",
                "candidate_seed_source",
                "base_cohort",
                "candidate_cohort",
                "base_log",
                "candidate_log",
            ],
            seed_rows,
        )
    )
    lines.extend(["## Topology / Timing Summary", ""])
    lines.append("Descriptive grouping only: profile labels do not replace topology semantics.")
    lines.append("")
    lines.extend(_markdown_table(["timing_group", "profile_ids"], timing_rows))
    lines.extend(
        _markdown_table(
            [
                "profile_id",
                "scenario",
                "label",
                "x",
                "y",
                "z",
                "cycle",
                "gap",
                "phase",
                "observer",
                "noise_seed",
                "launch_args",
                "notes",
            ],
            profile_rows,
        )
    )
    lines.extend(["## Anti-Claims", ""])
    for claim in _governance_block()["anti_claims"]:
        lines.append(f"- {claim}")
    lines.append("")
    return "\n".join(lines)


def render_dashboard_html(markdown_text: str) -> str:
    """Render dashboard markdown as deterministic standalone HTML."""

    def _cells(line: str) -> list[str]:
        return [c.strip() for c in line.strip().strip("|").split("|")]

    lines = markdown_text.splitlines()
    body: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("# "):
            body.append(f"<h1>{_html_cell(line[2:])}</h1>")
        elif line.startswith("## "):
            body.append(f"<h2>{_html_cell(line[3:])}</h2>")
        elif line.startswith("**Governance:**"):
            body.append(f"<p><strong>{_html_cell(line)}</strong></p>")
        elif line.startswith("- "):
            items = []
            while i < len(lines) and lines[i].startswith("- "):
                items.append(f"<li>{_html_cell(lines[i][2:])}</li>")
                i += 1
            body.append("<ul>" + "".join(items) + "</ul>")
            continue
        elif line.startswith("| ") and i + 1 < len(lines) and lines[i + 1].startswith("| "):
            headers = _cells(line)
            i += 2
            rows: list[list[str]] = []
            while i < len(lines) and lines[i].startswith("| "):
                rows.append(_cells(lines[i]))
                i += 1
            body.append(_html_table(headers, rows))
            continue
        elif line.strip():
            body.append(f"<p>{_html_cell(line)}</p>")
        i += 1

    return (
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Replay Reviewer Static Dashboard</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:1180px;margin:2rem auto;line-height:1.45;}"
        ".banner{border:1px solid #999;padding:1rem;margin-bottom:1rem;}"
        "table{border-collapse:collapse;width:100%;margin:0.75rem 0;}"
        "th,td{border:1px solid #aaa;padding:0.35rem;text-align:left;vertical-align:top;}"
        "th{font-weight:bold;}"
        "</style></head><body>"
        "<div class=\"banner\"><strong>Non-authoritative reviewer dashboard.</strong> "
        + html.escape(NON_AUTHORITATIVE_NOTICE)
        + "</div>"
        + "".join(body)
        + "</body></html>\n"
    )


def _cmd_bundle(args: argparse.Namespace) -> int:
    payload = build_evidence_bundle(args.log, meta_path=args.meta, evaluation_row_path=args.evaluation_row)
    _write_json(args.out_json, payload)
    print(f"Wrote {args.out_json.resolve()}")
    return 0


def _cmd_single_run_report(args: argparse.Namespace) -> int:
    payload = build_single_run_report(args.log, meta_path=args.meta)
    _write_json(args.out_json, payload)
    print(f"Wrote {args.out_json.resolve()}")
    return 0


def _cmd_paired_comparison(args: argparse.Namespace) -> int:
    payload = build_matched_seed_report(
        args.baseline_csv,
        args.candidate_csv,
        miss_delta=args.miss_delta,
        tint_delta=args.tint_delta,
    )
    _write_json(args.out_json, payload)
    print(f"Wrote {args.out_json.resolve()}")
    return 0


def _cmd_topology_index(args: argparse.Namespace) -> int:
    payload = build_topology_index(args.profiles_csv, summary_csv=args.summary_csv)
    _write_json(args.out_json, payload)
    print(f"Wrote {args.out_json.resolve()}")
    return 0


def _cmd_governance_lint(args: argparse.Namespace) -> int:
    payload = _read_json(args.artifact_json)
    result = lint_governance_artifact(payload)
    if args.out_json:
        _write_json(args.out_json, result)
        print(f"Wrote {args.out_json.resolve()}")
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["ok"] else 1


def _cmd_static_report(args: argparse.Namespace) -> int:
    payload = _read_json(args.artifact_json)
    markdown_text = render_static_markdown(payload)
    args.out_markdown.parent.mkdir(parents=True, exist_ok=True)
    args.out_markdown.write_text(markdown_text, encoding="utf-8")
    print(f"Wrote {args.out_markdown.resolve()}")
    if args.out_html:
        args.out_html.parent.mkdir(parents=True, exist_ok=True)
        args.out_html.write_text(render_static_html(markdown_text), encoding="utf-8")
        print(f"Wrote {args.out_html.resolve()}")
    return 0


def _cmd_dashboard(args: argparse.Namespace) -> int:
    input_paths = {
        "single_run_json": str(args.single_run_json) if args.single_run_json else "",
        "paired_comparison_json": str(args.paired_comparison_json) if args.paired_comparison_json else "",
        "topology_index_json": str(args.topology_index_json) if args.topology_index_json else "",
        "governance_lint_json": str(args.governance_lint_json) if args.governance_lint_json else "",
    }
    markdown_text = render_dashboard_markdown(
        single_run=_read_json(args.single_run_json),
        paired_comparison=_read_json(args.paired_comparison_json),
        topology_index=_read_json(args.topology_index_json),
        governance_lint=_read_json(args.governance_lint_json),
        input_paths={k: v for k, v in input_paths.items() if v},
    )
    args.out_html.parent.mkdir(parents=True, exist_ok=True)
    args.out_html.write_text(render_dashboard_html(markdown_text), encoding="utf-8")
    print(f"Wrote {args.out_html.resolve()}")
    if args.out_markdown:
        args.out_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.out_markdown.write_text(markdown_text, encoding="utf-8")
        print(f"Wrote {args.out_markdown.resolve()}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_bundle = sub.add_parser("bundle", help="Create a per-run replay evidence bundle JSON.")
    p_bundle.add_argument("log", type=Path)
    p_bundle.add_argument("--meta", type=Path, default=None)
    p_bundle.add_argument("--evaluation-row", type=Path, default=None)
    p_bundle.add_argument("--out-json", type=Path, required=True)
    p_bundle.set_defaults(func=_cmd_bundle)

    p_single = sub.add_parser("single-run-report", help="Create bundle + divergence trace + lifecycle timeline JSON.")
    p_single.add_argument("log", type=Path)
    p_single.add_argument("--meta", type=Path, default=None)
    p_single.add_argument("--out-json", type=Path, required=True)
    p_single.set_defaults(func=_cmd_single_run_report)

    p_pair = sub.add_parser("paired-comparison", help="Create a matched-seed comparison report JSON.")
    p_pair.add_argument("baseline_csv", type=Path)
    p_pair.add_argument("candidate_csv", type=Path)
    p_pair.add_argument("--out-json", type=Path, required=True)
    p_pair.add_argument("--miss-delta", type=float, default=0.02)
    p_pair.add_argument("--tint-delta", type=float, default=0.05)
    p_pair.set_defaults(func=_cmd_paired_comparison)

    p_topo = sub.add_parser("topology-index", help="Create a topology/timing analytics index from profile CSVs.")
    p_topo.add_argument("profiles_csv", type=Path)
    p_topo.add_argument("--summary-csv", type=Path, default=None)
    p_topo.add_argument("--out-json", type=Path, required=True)
    p_topo.set_defaults(func=_cmd_topology_index)

    p_lint = sub.add_parser("governance-lint", help="Validate governance labels on a derived artifact.")
    p_lint.add_argument("artifact_json", type=Path)
    p_lint.add_argument("--out-json", type=Path, default=None)
    p_lint.set_defaults(func=_cmd_governance_lint)

    p_static = sub.add_parser("static-report", help="Render a derived artifact as static markdown/html.")
    p_static.add_argument("artifact_json", type=Path)
    p_static.add_argument("--out-markdown", type=Path, required=True)
    p_static.add_argument("--out-html", type=Path, default=None)
    p_static.set_defaults(func=_cmd_static_report)

    p_dashboard = sub.add_parser("dashboard", help="Render a static reviewer dashboard from existing JSON artifacts.")
    p_dashboard.add_argument("--single-run-json", type=Path, default=None)
    p_dashboard.add_argument("--paired-comparison-json", type=Path, default=None)
    p_dashboard.add_argument("--topology-index-json", type=Path, default=None)
    p_dashboard.add_argument("--governance-lint-json", type=Path, default=None)
    p_dashboard.add_argument("--out-html", type=Path, required=True)
    p_dashboard.add_argument("--out-markdown", type=Path, default=None)
    p_dashboard.set_defaults(func=_cmd_dashboard)

    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
