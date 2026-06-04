#!/usr/bin/env python3
"""Map RT layout MC handoff artifacts to Planning-compatible result-ref payloads (CLI only).

Maintainer paste helper only: does not import into Planning UI, mutate runtime, or call RT bridge.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import rt_layout_mc_execute as layout_mc

PLANNING_MC_RESULT_REF_SCHEMA_VERSION = "rt_planning_mc_result_ref_v1"
LAYOUT_IDENTIFIER_FIELDS = ("job_id", "geometry_id", "source_layout_id", "cohort")


def layout_mc_result_id(job_id: str) -> str:
    safe = layout_mc._safe_id(job_id)
    return f"rt_mc_result:layout_mc:{safe}"


def _require_non_empty_string(data: dict[str, Any], field: str, *, label: str) -> str:
    value = str(data.get(field) or "").strip()
    if not value:
        raise ValueError(f"{label} {field} must be a non-empty string")
    return value


def audit_layout_identifier_propagation(
    *,
    manifest: dict[str, Any],
    result_summary: dict[str, Any] | None = None,
    result_link: dict[str, Any] | None = None,
    status: dict[str, Any] | None = None,
) -> list[str]:
    """Return issues when layout MC identifiers diverge across job artifacts."""
    issues: list[str] = []
    try:
        expected = {field: _require_non_empty_string(manifest, field, label="manifest") for field in LAYOUT_IDENTIFIER_FIELDS}
    except ValueError as exc:
        return [str(exc)]

    expected["run_count"] = manifest.get("run_count")

    for label, doc in (
        ("result_summary", result_summary),
        ("result_link", result_link),
    ):
        if doc is None:
            continue
        for field in LAYOUT_IDENTIFIER_FIELDS:
            if doc.get(field) != expected[field]:
                issues.append(
                    f"{label} {field} {doc.get(field)!r} does not match manifest {expected[field]!r}"
                )
        if expected["run_count"] is not None and doc.get("run_count") != expected["run_count"]:
            if label == "result_summary" and doc.get("run_count") != expected["run_count"]:
                issues.append(
                    f"{label} run_count {doc.get('run_count')!r} does not match manifest "
                    f"{expected['run_count']!r}"
                )

    if status is not None:
        if status.get("job_id") != expected["job_id"]:
            issues.append(
                f"status job_id {status.get('job_id')!r} does not match manifest {expected['job_id']!r}"
            )
        for field in LAYOUT_IDENTIFIER_FIELDS:
            if field == "job_id":
                continue
            if field in status and status.get(field) != expected[field]:
                issues.append(
                    f"status {field} {status.get(field)!r} does not match manifest {expected[field]!r}"
                )

    return issues


def load_layout_job_artifacts(job_dir: Path) -> dict[str, Any]:
    manifest = layout_mc._load_manifest_ref(job_dir)
    status = layout_mc._load_status_ref(job_dir)
    summary_path = job_dir / layout_mc.RESULT_SUMMARY_FILENAME
    link_path = job_dir / layout_mc.RESULT_LINK_FILENAME
    if not summary_path.is_file():
        raise FileNotFoundError(f"missing result summary: {summary_path}")
    if not link_path.is_file():
        raise FileNotFoundError(f"missing result link: {link_path}")
    result_summary = layout_mc._read_json_object(summary_path)
    result_link = layout_mc._read_json_object(link_path)
    if result_summary.get("schema_version") != layout_mc.RESULT_SUMMARY_SCHEMA_VERSION:
        raise ValueError(
            f"result_summary schema_version must be {layout_mc.RESULT_SUMMARY_SCHEMA_VERSION!r}"
        )
    if result_link.get("schema_version") != layout_mc.RESULT_LINK_SCHEMA_VERSION:
        raise ValueError(f"result_link schema_version must be {layout_mc.RESULT_LINK_SCHEMA_VERSION!r}")
    return {
        "manifest": manifest,
        "status": status,
        "result_summary": result_summary,
        "result_link": result_link,
    }


def _planning_summary_from_layout_mc_metrics(metrics: dict[str, Any]) -> dict[str, float]:
    summary: dict[str, float] = {}
    success_rate = layout_mc._finite_number(metrics.get("success_rate"))
    if success_rate is not None:
        summary["success_rate"] = success_rate
    miss_p95 = layout_mc._finite_number(metrics.get("miss_distance_m_p95"))
    if miss_p95 is not None:
        summary["miss_distance_p95"] = miss_p95
    intercept_mean = layout_mc._finite_number(metrics.get("intercept_time_s_mean"))
    if intercept_mean is not None:
        summary["intercept_time_mean"] = intercept_mean
    return summary


def build_planning_mc_result_ref_payload(
    *,
    result_link: dict[str, Any],
    result_summary: dict[str, Any],
    linked_package_id: str,
    linked_planning_geometry_id: str,
    imported_utc: str | None = None,
) -> dict[str, Any]:
    """Build rt_planning_mc_result_ref_v1 JSON from layout MC handoff artifacts.

    Planning snapshot/package ids must be supplied by the maintainer; layout geometry_id
    (rt_layout:*) is not auto-mapped to planning_geometry_id (rt_planning:*).
    """
    issues = audit_layout_identifier_propagation(
        manifest={
            "job_id": result_link["job_id"],
            "geometry_id": result_link["geometry_id"],
            "source_layout_id": result_link["source_layout_id"],
            "cohort": result_link["cohort"],
            "run_count": result_summary.get("run_count"),
        },
        result_summary=result_summary,
        result_link=result_link,
    )
    if issues:
        raise ValueError("layout handoff identifier mismatch: " + "; ".join(issues))

    job_id = _require_non_empty_string(result_link, "job_id", label="result_link")
    package_id = linked_package_id.strip()
    planning_geometry_id = linked_planning_geometry_id.strip()
    if not package_id:
        raise ValueError("linked_package_id must be a non-empty string")
    if not planning_geometry_id:
        raise ValueError("linked_planning_geometry_id must be a non-empty string")

    metrics = result_summary.get("mc_metrics")
    if not isinstance(metrics, dict):
        metrics = {}

    payload: dict[str, Any] = {
        "schema_version": PLANNING_MC_RESULT_REF_SCHEMA_VERSION,
        "linked_package_id": package_id,
        "linked_planning_geometry_id": planning_geometry_id,
        "mc_run_label": job_id,
        "mc_result_id": layout_mc_result_id(job_id),
        "imported_utc": imported_utc
        or str(result_summary.get("completed_utc") or result_link.get("linked_utc") or "").strip()
        or layout_mc._utc_now(),
    }
    summary = _planning_summary_from_layout_mc_metrics(metrics)
    if summary:
        payload["summary"] = summary
    return payload


def cmd_map(args: argparse.Namespace) -> int:
    try:
        artifacts = load_layout_job_artifacts(args.job_dir)
        propagation_issues = audit_layout_identifier_propagation(
            manifest=artifacts["manifest"],
            result_summary=artifacts["result_summary"],
            result_link=artifacts["result_link"],
            status=artifacts["status"],
        )
        if propagation_issues:
            raise ValueError("; ".join(propagation_issues))
        payload = build_planning_mc_result_ref_payload(
            result_link=artifacts["result_link"],
            result_summary=artifacts["result_summary"],
            linked_package_id=args.linked_package_id,
            linked_planning_geometry_id=args.linked_planning_geometry_id,
            imported_utc=args.imported_utc,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"rt_layout_mc_planning_ref map: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Map layout MC handoff artifacts to Planning result-ref JSON (maintainer only)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    map_cmd = sub.add_parser(
        "map",
        help="Emit rt_planning_mc_result_ref_v1 JSON from result_link.json + result_summary.json.",
    )
    map_cmd.add_argument("job_dir", type=Path, help="Completed layout MC job directory.")
    map_cmd.add_argument(
        "--linked-package-id",
        required=True,
        help="Planning package link id (rt_planning_package:{planning_snapshot_id}).",
    )
    map_cmd.add_argument(
        "--linked-planning-geometry-id",
        required=True,
        help="Planning geometry id (rt_planning:sha256:...); not inferred from layout geometry_id.",
    )
    map_cmd.add_argument("--imported-utc", default=None)
    map_cmd.set_defaults(func=cmd_map)

    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
