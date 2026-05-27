#!/usr/bin/env python3
"""Maintainer-only RT experiment analytics derive (PLAT-RT-F1).

Reads rt_experiment_manifest_v1 (+ optional rt_experiment_batch_v1) and writes
rt_experiment_analytics_report_v1. Explanatory only — not operational authority.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

ANALYTICS_GOVERNANCE_BANNER = (
    "RT ANALYTICS — derived summaries only; not operational authority"
)

FORBIDDEN_ROLLUP_KEYS = frozenset({"success_rate", "readiness", "winner"})


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _short_id(value: str | None) -> str | None:
    if not value:
        return None
    return value[:8] if len(value) > 8 else value


def _load_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must be a JSON object")
    return data


def _load_batch(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix in {".yaml", ".yml"}:
        if yaml is None:
            raise RuntimeError("PyYAML required for YAML batch specs")
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict) or data.get("schema") != "rt_experiment_batch_v1":
        raise ValueError("batch must be rt_experiment_batch_v1")
    return data


def _batch_row(batch: dict[str, Any] | None, run_id: str) -> dict[str, Any] | None:
    if not batch:
        return None
    for row in batch.get("runs") or []:
        if isinstance(row, dict) and row.get("run_id") == run_id:
            return row
    return None


def _record(obj: Any) -> dict[str, Any]:
    return obj if isinstance(obj, dict) else {}


def _normalization_status_ref(repo_root: Path, staging_ref: str | None) -> str:
    if not staging_ref:
        return "unavailable"
    staging = repo_root / staging_ref
    manifest_path = staging / "normalized_manifest.json"
    if not manifest_path.is_file():
        return "unavailable"
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return "unavailable"
    if isinstance(data, dict):
        status = data.get("normalization_status")
        if isinstance(status, str):
            return status
    return "unavailable"


def _per_run(
    run: dict[str, Any],
    batch: dict[str, Any] | None,
    repo_root: Path,
) -> dict[str, Any]:
    batch_row = _batch_row(batch, str(run["run_id"]))
    snapshot = _record(run.get("snapshot"))
    tactical = _record(snapshot.get("tactical_state"))
    world = _record(snapshot.get("world_summary"))
    annex = _record(run.get("tactical_annex_summary"))
    terrain = _record(snapshot.get("terrain_context"))

    entity_count = world.get("entity_count")
    tti = tactical.get("tti_s")

    return {
        "run_id": run["run_id"],
        "label": run.get("label") or run["run_id"],
        "session_id_short": _short_id(str(run.get("session_id") or "")) or "—",
        "recorded_at_utc": run.get("recorded_at_utc") or "",
        "dwell_s": (batch_row or {}).get("dwell_s") or (batch or {}).get("default_dwell_s"),
        "template_id": (batch_row or {}).get("template_id"),
        "tactical_mode_hint": (batch_row or {}).get("tactical_mode_hint"),
        "entity_count": entity_count if isinstance(entity_count, (int, float)) else None,
        "adapter_mode": world.get("adapter_mode") if isinstance(world.get("adapter_mode"), str) else None,
        "sync_health": world.get("sync_health") if isinstance(world.get("sync_health"), str) else None,
        "lifecycle_state": snapshot.get("lifecycle_state"),
        "tactical_mode": tactical.get("tactical_mode")
        if isinstance(tactical.get("tactical_mode"), str)
        else None,
        "selected_id_short": _short_id(
            str(
                tactical.get("selected_target_id")
                or tactical.get("selected_interceptor_id")
                or ""
            )
        ),
        "assigned_id_short": _short_id(
            str(
                tactical.get("assigned_target_id")
                or tactical.get("assigned_interceptor_id")
                or ""
            )
        ),
        "tti_s": float(tti) if isinstance(tti, (int, float)) else None,
        "autonomous_loop_status": tactical.get("autonomous_loop_status")
        if isinstance(tactical.get("autonomous_loop_status"), str)
        else None,
        "has_capture": bool(run.get("capture_candidate_id")),
        "capture_candidate_id": run.get("capture_candidate_id"),
        "capture_staging_ref": run.get("capture_staging_ref"),
        "normalization_status_ref": _normalization_status_ref(
            repo_root, run.get("capture_staging_ref")
        ),
        "annex_timeline_counts": annex.get("timeline_counts"),
        "annex_final_mode": annex.get("final_tactical_mode"),
        "terrain_nearest_ridge": terrain.get("nearest_ridge"),
    }


def _compare_side(run: dict[str, Any]) -> dict[str, Any]:
    snapshot = _record(run.get("snapshot"))
    return {
        "tactical": _record(snapshot.get("tactical_state")),
        "world": _record(snapshot.get("world_summary")),
    }


def _compare_badges(a: dict[str, Any], b: dict[str, Any]) -> list[dict[str, str]]:
    ta = a["tactical"]
    tb = b["tactical"]
    badges: list[dict[str, str]] = []
    if (ta.get("tactical_mode") or "") != (tb.get("tactical_mode") or ""):
        badges.append({"id": "mode_changed", "label": "mode_changed"})
    assign_a = ta.get("assigned_target_id") or ta.get("assigned_interceptor_id")
    assign_b = tb.get("assigned_target_id") or tb.get("assigned_interceptor_id")
    if (assign_a or "") != (assign_b or ""):
        badges.append({"id": "assignment_changed", "label": "assignment_changed"})
    tti_a, tti_b = ta.get("tti_s"), tb.get("tti_s")
    if (
        isinstance(tti_a, (int, float))
        and isinstance(tti_b, (int, float))
        and abs(float(tti_a) - float(tti_b)) > 0.05
    ):
        badges.append({"id": "tti_delta", "label": "tti_delta"})
    if (ta.get("autonomous_loop_status") or "") != (tb.get("autonomous_loop_status") or ""):
        badges.append({"id": "pause_resume_delta", "label": "pause_resume_delta"})
    return badges


def _rollup(per_run: list[dict[str, Any]]) -> dict[str, Any]:
    mode_counts: dict[str, int] = {}
    template_ids: set[str] = set()
    capture_count = 0
    for row in per_run:
        if row.get("has_capture"):
            capture_count += 1
        mode = row.get("tactical_mode") or "unknown"
        mode_counts[str(mode)] = mode_counts.get(str(mode), 0) + 1
        tid = row.get("template_id")
        if isinstance(tid, str) and tid:
            template_ids.add(tid)
    rollup = {
        "run_count": len(per_run),
        "capture_count": capture_count,
        "mode_counts": mode_counts,
        "template_ids_used": sorted(template_ids),
    }
    for key in rollup:
        if key in FORBIDDEN_ROLLUP_KEYS:
            raise ValueError(f"forbidden rollup key: {key}")
    return rollup


def derive_analytics(
    manifest: dict[str, Any],
    batch: dict[str, Any] | None = None,
    *,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    if manifest.get("schema") != "rt_experiment_manifest_v1":
        raise ValueError("manifest must be rt_experiment_manifest_v1")
    root = repo_root or _REPO_ROOT
    runs = sorted(manifest.get("runs") or [], key=lambda r: str(r.get("run_id", "")))
    per_run = [_per_run(r, batch, root) for r in runs if isinstance(r, dict)]
    compare_pairs: list[dict[str, Any]] = []
    for i, run_a in enumerate(runs):
        if not isinstance(run_a, dict):
            continue
        for run_b in runs[i + 1 :]:
            if not isinstance(run_b, dict):
                continue
            compare_pairs.append(
                {
                    "run_id_a": run_a["run_id"],
                    "run_id_b": run_b["run_id"],
                    "badges": _compare_badges(_compare_side(run_a), _compare_side(run_b)),
                }
            )
    return {
        "schema": "rt_experiment_analytics_report_v1",
        "experiment_id": manifest.get("experiment_id") or "unknown",
        "governance_banner": ANALYTICS_GOVERNANCE_BANNER,
        "per_run": per_run,
        "compare_pairs": compare_pairs,
        "rollup": _rollup(per_run),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive RT experiment analytics report")
    parser.add_argument("--manifest", required=True, type=Path, help="rt_experiment_manifest_v1 JSON")
    parser.add_argument("--batch", type=Path, help="optional rt_experiment_batch_v1")
    parser.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    parser.add_argument("--out", type=Path, help="write report JSON (adds derived_at_utc)")
    args = parser.parse_args(argv)

    manifest = _load_json(args.manifest)
    batch = _load_batch(args.batch) if args.batch else None
    report = derive_analytics(manifest, batch, repo_root=args.repo_root.resolve())
    report["derived_at_utc"] = _utc_now()

    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
