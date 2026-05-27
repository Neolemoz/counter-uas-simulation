#!/usr/bin/env python3
"""Derive rt_experiment_metrics_report_v1 (PLAT-RT-F5).

Reads rt_experiment_manifest_v1 and rt_experiment_analytics_report_v1 (or derives F1
from manifest + optional batch). Mirrors platform/rt-sandbox-ui metricsDerive.ts.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.rt import rt_experiment_analytics as analytics  # noqa: E402
from scripts.rt import rt_experiment_spec_compile as spec_compile  # noqa: E402

METRICS_GOVERNANCE_BANNER = (
    "RT EXPERIMENT METRICS — derived summaries only; not operational authority"
)

FORBIDDEN_ROLLUP_KEYS = frozenset(
    {"success_rate", "winner", "best_run", "readiness_index"},
)


def _record(obj: Any) -> dict[str, Any]:
    return obj if isinstance(obj, dict) else {}


def _terrain_from_run(run: dict[str, Any]) -> dict[str, Any]:
    snap = _record(run.get("snapshot"))
    terrain = _record(snap.get("terrain_context"))
    vis = _record(run.get("visibility_context"))
    f4_preset = vis.get("f4_layer_preset")
    return {
        "nearest_ridge": terrain.get("nearest_ridge"),
        "elevation_band": terrain.get("elevation_band"),
        "terrain_profile_ref": run.get("terrain_profile_ref") or terrain.get("terrain_profile_ref"),
        "f4_layers_enabled": [f4_preset] if isinstance(f4_preset, str) else [],
        "los_cognition_label": vis.get("los_cognition_label")
        if vis.get("los_cognition_label") is not None
        else terrain.get("visibility_hint"),
        "occlusion_marker_count": vis.get("occlusion_marker_count"),
    }


def _experiment_class_for_run(run: dict[str, Any], spec: dict[str, Any] | None) -> str | None:
    ec = run.get("experiment_class")
    if ec:
        return str(ec)
    if spec:
        return str(spec.get("experiment_class")) if spec.get("experiment_class") else None
    return None


def _axis_signature(coords: dict[str, str] | None) -> str | None:
    if not coords:
        return None
    return ";".join(f"{k}={coords[k]}" for k in sorted(coords))


def _count_map_inc(counts: dict[str, int], key: str | None) -> None:
    k = key if key is not None else "unknown"
    counts[k] = counts.get(k, 0) + 1


def _cartesian_product_size(axes: list[dict[str, Any]]) -> int:
    size = 1
    for axis in axes:
        values = axis.get("values") or []
        size *= len(values)
    return size


def _compare_side_from_run(run: dict[str, Any]) -> dict[str, Any]:
    snap = _record(run.get("snapshot"))
    tactical = _record(snap.get("tactical_state"))
    return {
        "tactical": tactical,
        "world": _record(snap.get("world_summary")),
    }


def _compare_badges(a: dict[str, Any], b: dict[str, Any]) -> list[dict[str, str]]:
    ta = a["tactical"]
    tb = b["tactical"]
    badges: list[dict[str, str]] = []
    if (ta.get("tactical_mode") or "") != (tb.get("tactical_mode") or ""):
        badges.append(
            {
                "id": "mode_changed",
                "label": "mode_changed",
                "detail": f"{ta.get('tactical_mode') or '—'} → {tb.get('tactical_mode') or '—'}",
            }
        )
    assign_a = ta.get("assigned_target_id") or ta.get("assigned_interceptor_id")
    assign_b = tb.get("assigned_target_id") or tb.get("assigned_interceptor_id")
    if (assign_a or "") != (assign_b or ""):
        badges.append({"id": "assignment_changed", "label": "assignment_changed"})
    tti_a, tti_b = ta.get("tti_s"), tb.get("tti_s")
    if isinstance(tti_a, (int, float)) and isinstance(tti_b, (int, float)):
        if abs(float(tti_a) - float(tti_b)) > 0.05:
            badges.append(
                {
                    "id": "tti_delta",
                    "label": "tti_delta",
                    "detail": f"{float(tti_a):.1f}s vs {float(tti_b):.1f}s",
                }
            )
    if (ta.get("autonomous_loop_status") or "") != (tb.get("autonomous_loop_status") or ""):
        badges.append({"id": "pause_resume_delta", "label": "pause_resume_delta"})
    return badges


def per_run_extended_from_run(
    run: dict[str, Any],
    f1_row: dict[str, Any],
    prior_assigned: str | None,
    spec: dict[str, Any] | None,
) -> dict[str, Any]:
    terrain = _terrain_from_run(run)
    assigned = f1_row.get("assigned_id_short")
    assign_delta = (
        prior_assigned is not None
        and assigned is not None
        and prior_assigned != assigned
    )
    annex = _record(run.get("tactical_annex_summary"))
    counts = _record(annex.get("timeline_counts"))
    pause_count = counts.get("pause_resume_transitions")
    matrix_coords = run.get("matrix_coords")
    if matrix_coords is not None and not isinstance(matrix_coords, dict):
        matrix_coords = None

    return {
        "run_id": run["run_id"],
        "experiment_class": _experiment_class_for_run(run, spec),
        "spec_fingerprint": run.get("spec_fingerprint"),
        "matrix_coords": matrix_coords,
        "axis_signature": _axis_signature(matrix_coords),
        "repeat_group_id": run.get("repeat_group_id"),
        "repeat_index": run.get("repeat_index"),
        "terrain_profile_ref": terrain.get("terrain_profile_ref"),
        "nearest_ridge": terrain.get("nearest_ridge"),
        "elevation_band": terrain.get("elevation_band"),
        "f4_layers_enabled": terrain["f4_layers_enabled"],
        "los_cognition_label": terrain.get("los_cognition_label"),
        "occlusion_marker_count": terrain.get("occlusion_marker_count"),
        "mode_at_capture": f1_row.get("tactical_mode"),
        "assign_delta_from_prior": assign_delta,
        "autonomous_pause_count": pause_count if isinstance(pause_count, int) else None,
        "handoff_eligibility_hint": "unknown" if f1_row.get("has_capture") else "ineligible",
    }


def rollup_extended(
    per_run: list[dict[str, Any]],
    f1_report: dict[str, Any],
    spec: dict[str, Any] | None,
) -> dict[str, Any]:
    counts_by_class: dict[str, int] = {}
    ridge_counts: dict[str, int] = {}
    band_counts: dict[str, int] = {}
    los_label_counts: dict[str, int] = {}
    annex_event_totals: dict[str, int] = {}
    assign_change_count = 0
    tti_present_count = 0
    fingerprint_groups: dict[str, dict[str, Any]] = {}

    f1_by_id = {r["run_id"]: r for r in f1_report.get("per_run") or []}

    for row in per_run:
        _count_map_inc(counts_by_class, row.get("experiment_class"))
        _count_map_inc(ridge_counts, row.get("nearest_ridge"))
        _count_map_inc(band_counts, row.get("elevation_band"))
        _count_map_inc(los_label_counts, row.get("los_cognition_label"))
        if row.get("assign_delta_from_prior"):
            assign_change_count += 1
        f1 = f1_by_id.get(row["run_id"])
        if f1 and f1.get("tti_s") is not None:
            tti_present_count += 1
        fp = row.get("spec_fingerprint") or "unknown"
        if fp not in fingerprint_groups:
            fingerprint_groups[fp] = {
                "run_count": 0,
                "capture_count": 0,
                "normalization_status_counts": {},
            }
        g = fingerprint_groups[fp]
        g["run_count"] += 1
        if f1 and f1.get("has_capture"):
            g["capture_count"] += 1
        norm = (f1 or {}).get("normalization_status_ref") or "unavailable"
        g["normalization_status_counts"][norm] = (
            g["normalization_status_counts"].get(norm, 0) + 1
        )

    for row in f1_report.get("per_run") or []:
        counts = row.get("annex_timeline_counts")
        if not isinstance(counts, dict):
            continue
        for k, v in counts.items():
            if isinstance(v, int):
                annex_event_totals[k] = annex_event_totals.get(k, 0) + v

    expected_cells = 0
    if spec and spec.get("compile_strategy") == "cartesian" and spec.get("matrix_axes"):
        expected_cells = _cartesian_product_size(spec["matrix_axes"])
    elif spec:
        me = _record(spec.get("manifest_expectation"))
        min_run = me.get("min_run_count")
        if isinstance(min_run, int):
            expected_cells = min_run

    populated_cells = sum(1 for r in per_run if r.get("matrix_coords") is not None)
    missing_cells = max(0, expected_cells - populated_cells)

    mode_counts = dict(_record(f1_report.get("rollup")).get("mode_counts") or {})
    for key in mode_counts:
        if key in FORBIDDEN_ROLLUP_KEYS:
            raise ValueError(f"forbidden rollup key: {key}")

    return {
        "class_rollup": {"counts_by_class": counts_by_class},
        "terrain_rollup": {"ridge_counts": ridge_counts, "band_counts": band_counts},
        "visibility_rollup": {"los_label_counts": los_label_counts},
        "tactical_rollup": {
            "mode_counts": mode_counts,
            "assign_change_count": assign_change_count,
            "tti_present_count": tti_present_count,
            "annex_event_totals": annex_event_totals,
        },
        "repeatability_rollup": {
            "fingerprints": [
                {
                    "spec_fingerprint": fp,
                    "run_count": g["run_count"],
                    "capture_count": g["capture_count"],
                    "normalization_status_counts": g["normalization_status_counts"],
                }
                for fp, g in sorted(fingerprint_groups.items(), key=lambda x: x[0])
            ],
        },
        "matrix_rollup": {
            "expected_cells": expected_cells,
            "populated_cells": populated_cells,
            "missing_cells": missing_cells,
        },
    }


def _matrix_axis_diff_count(
    a: dict[str, str] | None,
    b: dict[str, str] | None,
) -> int:
    if not a or not b:
        return -1
    keys = set(a) | set(b)
    diffs = sum(1 for k in keys if (a.get(k) or "") != (b.get(k) or ""))
    return diffs


def _annex_total_count(run_id: str, f1_report: dict[str, Any]) -> int:
    for row in f1_report.get("per_run") or []:
        if row.get("run_id") != run_id:
            continue
        counts = row.get("annex_timeline_counts")
        if not isinstance(counts, dict):
            return 0
        return sum(v for v in counts.values() if isinstance(v, int))
    return 0


def build_extended_compare_pairs(
    manifest: dict[str, Any],
    per_run: list[dict[str, Any]],
    f1_report: dict[str, Any],
) -> list[dict[str, Any]]:
    runs = sorted(
        [r for r in manifest.get("runs") or [] if isinstance(r, dict)],
        key=lambda r: str(r.get("run_id", "")),
    )
    ext_by_id = {r["run_id"]: r for r in per_run}
    pairs: list[dict[str, Any]] = []

    for i, run_a in enumerate(runs):
        for run_b in runs[i + 1 :]:
            ext_a = ext_by_id.get(run_a["run_id"])
            ext_b = ext_by_id.get(run_b["run_id"])
            if not ext_a or not ext_b:
                continue

            badges = _compare_badges(
                _compare_side_from_run(run_a),
                _compare_side_from_run(run_b),
            )

            if ext_a.get("experiment_class") != ext_b.get("experiment_class"):
                badges.append(
                    {
                        "id": "class_mismatch",
                        "label": "class_mismatch",
                        "detail": f"{ext_a.get('experiment_class')} vs {ext_b.get('experiment_class')}",
                    }
                )

            f1a = next(
                (r for r in f1_report.get("per_run") or [] if r.get("run_id") == run_a["run_id"]),
                None,
            )
            f1b = next(
                (r for r in f1_report.get("per_run") or [] if r.get("run_id") == run_b["run_id"]),
                None,
            )
            if bool(f1a and f1a.get("has_capture")) != bool(f1b and f1b.get("has_capture")):
                badges.append(
                    {
                        "id": "capture_asymmetric",
                        "label": "capture_asymmetric",
                        "detail": "capture presence differs",
                    }
                )

            if ext_a.get("nearest_ridge") != ext_b.get("nearest_ridge") or ext_a.get(
                "elevation_band"
            ) != ext_b.get("elevation_band"):
                badges.append(
                    {"id": "terrain_context_diff", "label": "terrain_context_diff"}
                )

            if ext_a.get("los_cognition_label") != ext_b.get("los_cognition_label"):
                badges.append(
                    {"id": "visibility_label_diff", "label": "visibility_label_diff"}
                )

            annex_a = _annex_total_count(run_a["run_id"], f1_report)
            annex_b = _annex_total_count(run_b["run_id"], f1_report)
            if annex_a != annex_b:
                badges.append(
                    {
                        "id": "annex_count_delta",
                        "label": "annex_count_delta",
                        "detail": f"{annex_a} vs {annex_b}",
                    }
                )

            axis_diffs = _matrix_axis_diff_count(
                ext_a.get("matrix_coords"),
                ext_b.get("matrix_coords"),
            )
            if axis_diffs == 1:
                badges.append({"id": "matrix_axis_diff", "label": "matrix_axis_diff"})

            pairs.append(
                {
                    "run_id_a": run_a["run_id"],
                    "run_id_b": run_b["run_id"],
                    "badges": badges,
                }
            )

    return sorted(pairs, key=lambda p: (p["run_id_a"], p["run_id_b"]))


def evaluate_handoff_eligibility(
    manifest: dict[str, Any],
    f1_report: dict[str, Any],
    *,
    repo_root: Path | None = None,
    maintainer_ack_pose_reviewed: bool = False,
    staging_reader: Any | None = None,
) -> dict[str, Any]:
    root = repo_root or _REPO_ROOT
    gates: list[dict[str, Any]] = []
    per_run_gates: list[dict[str, Any]] = []

    per_run = f1_report.get("per_run") or []
    capture_missing = sum(1 for r in per_run if not r.get("has_capture"))
    gates.append(
        {
            "id": "all_captures_present",
            "pass": capture_missing == 0,
            "detail": "all runs have capture"
            if capture_missing == 0
            else f"{capture_missing}/{len(per_run)} runs missing capture",
        }
    )

    norm_fail = False
    for row in per_run:
        status = row.get("normalization_status_ref")
        staging_ref = row.get("capture_staging_ref")
        if staging_reader and staging_ref:
            status = staging_reader(staging_ref) or "unavailable"
        if status != "normalized":
            norm_fail = True

    gates.append(
        {
            "id": "normalization_available",
            "pass": not norm_fail and all(r.get("has_capture") for r in per_run),
            "detail": "normalization not normalized for all captures" if norm_fail else "ok",
        }
    )

    gates.append(
        {
            "id": "lifecycle_importable",
            "pass": True,
            "detail": "lifecycle not evaluated in P0 derive",
        }
    )

    gates.append(
        {
            "id": "pose_cognition_ack",
            "pass": maintainer_ack_pose_reviewed,
            "detail": "acknowledged" if maintainer_ack_pose_reviewed else "not acknowledged",
        }
    )

    classes = {r.get("experiment_class") for r in manifest.get("runs") or [] if r.get("experiment_class")}
    gates.append(
        {
            "id": "no_class_mismatch",
            "pass": len(classes) <= 1,
            "detail": "single class" if len(classes) <= 1 else f"classes: {', '.join(sorted(classes))}",
        }
    )

    all_pass = all(g["pass"] for g in gates)
    experiment_level = "eligible" if all_pass else "ineligible"
    eligible_runs = sum(1 for r in per_run if r.get("has_capture"))
    if not all_pass and 0 < eligible_runs < len(per_run):
        experiment_level = "partial"

    for row in per_run:
        per_run_gates.append(
            {
                "run_id": row["run_id"],
                "eligible": bool(row.get("has_capture")),
                "gates": [
                    {
                        "id": "has_capture",
                        "pass": bool(row.get("has_capture")),
                        "detail": "capture present" if row.get("has_capture") else "no capture",
                    }
                ],
            }
        )

    return {
        "experiment_level": experiment_level,
        "gates": gates,
        "per_run_gates": per_run_gates,
    }


def derive_metrics(
    manifest: dict[str, Any],
    f1_report: dict[str, Any],
    *,
    spec: dict[str, Any] | None = None,
    repo_root: Path | None = None,
    maintainer_ack_pose_reviewed: bool = False,
) -> dict[str, Any]:
    if manifest.get("schema") != "rt_experiment_manifest_v1":
        raise ValueError("manifest must be rt_experiment_manifest_v1")
    if f1_report.get("schema") != "rt_experiment_analytics_report_v1":
        raise ValueError("analytics must be rt_experiment_analytics_report_v1")

    sorted_runs = sorted(
        [r for r in manifest.get("runs") or [] if isinstance(r, dict)],
        key=lambda r: str(r.get("run_id", "")),
    )
    f1_by_id = {r["run_id"]: r for r in f1_report.get("per_run") or []}

    per_run_extended: list[dict[str, Any]] = []
    prior_fp: str | None = None
    prior_assigned: str | None = None

    for run in sorted_runs:
        f1_row = f1_by_id.get(run["run_id"])
        if not f1_row:
            raise ValueError(f"missing f1 per_run for {run['run_id']}")
        fp = run.get("spec_fingerprint")
        prior = prior_assigned if fp and fp == prior_fp else None
        ext = per_run_extended_from_run(run, f1_row, prior, spec)
        per_run_extended.append(ext)
        prior_fp = str(fp) if fp else None
        prior_assigned = f1_row.get("assigned_id_short")

    classes = [r.get("experiment_class") for r in sorted_runs if r.get("experiment_class")]
    experiment_class = (
        classes[0] if classes else (spec.get("experiment_class") if spec else None) or "unknown"
    )

    manifest_for_pairs = {**manifest, "runs": sorted_runs}

    return {
        "schema": "rt_experiment_metrics_report_v1",
        "experiment_id": manifest.get("experiment_id") or "unknown",
        "experiment_class": experiment_class,
        "governance_banner": METRICS_GOVERNANCE_BANNER,
        "spec_fingerprint": per_run_extended[0].get("spec_fingerprint") if per_run_extended else None,
        "per_run_extended": per_run_extended,
        "compare_pairs_extended": build_extended_compare_pairs(
            manifest_for_pairs, per_run_extended, f1_report
        ),
        "rollup_extended": rollup_extended(per_run_extended, f1_report, spec),
        "handoff_eligibility": evaluate_handoff_eligibility(
            manifest_for_pairs,
            f1_report,
            repo_root=repo_root,
            maintainer_ack_pose_reviewed=maintainer_ack_pose_reviewed,
        ),
    }


def _load_spec(path: Path) -> dict[str, Any]:
    return spec_compile._load_spec(path)


def _load_analytics(path: Path) -> dict[str, Any]:
    data = analytics._load_json(path)
    if data.get("schema") != "rt_experiment_analytics_report_v1":
        raise ValueError("analytics must be rt_experiment_analytics_report_v1")
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive RT experiment metrics report")
    parser.add_argument("--manifest", required=True, type=Path, help="rt_experiment_manifest_v1 JSON")
    parser.add_argument("--batch", type=Path, help="optional rt_experiment_batch_v1 for F1 derive")
    parser.add_argument(
        "--analytics",
        type=Path,
        help="optional rt_experiment_analytics_report_v1 (skips F1 derive)",
    )
    parser.add_argument("--spec", type=Path, help="optional rt_experiment_spec_v1")
    parser.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    parser.add_argument(
        "--maintainer-ack-pose-reviewed",
        action="store_true",
        help="pass pose_cognition_ack gate",
    )
    parser.add_argument("--out", type=Path, help="write report JSON (adds derived_at_utc)")
    args = parser.parse_args(argv)

    manifest = analytics._load_json(args.manifest)
    batch = analytics._load_batch(args.batch) if args.batch else None
    spec = _load_spec(args.spec) if args.spec else None
    repo_root = args.repo_root.resolve()

    if args.analytics:
        f1_report = _load_analytics(args.analytics)
    else:
        f1_report = analytics.derive_analytics(manifest, batch, repo_root=repo_root)

    report = derive_metrics(
        manifest,
        f1_report,
        spec=spec,
        repo_root=repo_root,
        maintainer_ack_pose_reviewed=args.maintainer_ack_pose_reviewed,
    )
    report["derived_at_utc"] = analytics._utc_now()

    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
