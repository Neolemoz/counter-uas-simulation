#!/usr/bin/env python3
"""Derive rt_experiment_fidelity_metrics_report_v1 (PLAT-RT-F5b P2).

Reads rt_experiment_manifest_v1 and optional staging for fidelity_truth /
fidelity_pose_block. Mirrors platform/rt-sandbox-ui fidelityMetricsDerive.ts.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

FIDELITY_METRICS_GOVERNANCE_BANNER = (
    "RT EXPERIMENT FIDELITY METRICS — truth-attested summaries are sim-scoped; "
    "not SA replay or operational sensor authority"
)

POSE_TRUTH_DRIFT_COMPARE_EPSILON_M = 0.001

FORBIDDEN_ROLLUP_KEYS = frozenset(
    {"success_rate", "winner", "best_run", "readiness_index"},
)


def _record(obj: Any) -> dict[str, Any]:
    return obj if isinstance(obj, dict) else {}


def _sort_keys_deep(value: Any) -> Any:
    if isinstance(value, list):
        return [_sort_keys_deep(v) for v in value]
    if isinstance(value, dict):
        return {k: _sort_keys_deep(value[k]) for k in sorted(value)}
    return value


def compute_truth_fingerprint(
    fidelity_context: dict[str, Any] | None,
    truth_snapshot: dict[str, Any] | None,
) -> str | None:
    if not fidelity_context or not fidelity_context.get("enable_fidelity_coupling"):
        return None
    truth = _record(truth_snapshot)
    vis = _record(truth.get("visibility_truth"))
    dome = _record(truth.get("dome_truth"))
    payload = {
        "enable_fidelity_coupling": fidelity_context.get("enable_fidelity_coupling"),
        "adapter_mode": fidelity_context.get("adapter_mode"),
        "visibility_truth_ref": vis.get("ref"),
        "dome_truth_ref": dome.get("sensor_id"),
    }
    canonical = json.dumps(_sort_keys_deep(payload), separators=(",", ":"), sort_keys=True)
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
    return f"truth-fp-{digest}"


def _read_staging_json(repo_root: Path, ref: str | None) -> dict[str, Any] | None:
    if not ref:
        return None
    path = repo_root / ref
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except (OSError, json.JSONDecodeError):
        return None


def _read_truth_snapshot(run: dict[str, Any], repo_root: Path) -> dict[str, Any] | None:
    ctx = _record(run.get("fidelity_context"))
    ref = ctx.get("truth_snapshot_ref")
    return _read_staging_json(repo_root, str(ref) if ref else None)


def _read_pose_block(run: dict[str, Any], repo_root: Path) -> dict[str, Any] | None:
    staging_ref = run.get("capture_staging_ref")
    if not staging_ref:
        return None
    raw = _read_staging_json(repo_root, f"{staging_ref}/normalized_manifest.json")
    if not raw:
        return None
    block = raw.get("fidelity_pose_block")
    return block if isinstance(block, dict) else None


def _max_numeric(values: list[Any]) -> float | None:
    nums = [v for v in values if isinstance(v, (int, float))]
    return max(nums) if nums else None


def _attestation_status(
    truth: dict[str, Any] | None,
    pose_block: dict[str, Any] | None,
    coupling_on: bool,
) -> str:
    if not coupling_on:
        return "unavailable"
    for source in (truth, pose_block):
        if not source:
            continue
        status = source.get("attestation_status")
        if status in ("available", "stale"):
            return str(status)
    if truth or pose_block:
        return "unavailable"
    return "unavailable"


def per_run_fidelity_from_run(run: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    ctx = _record(run.get("fidelity_context"))
    coupling_on = bool(ctx.get("enable_fidelity_coupling"))
    truth = _read_truth_snapshot(run, repo_root)
    pose_block = _read_pose_block(run, repo_root)

    los_truth_label = truth.get("los_truth", {}).get("label") if coupling_on and truth else None
    vis_ctx = _record(run.get("visibility_context"))
    los_cognition_label = vis_ctx.get("los_cognition_label")

    cognition_truth_divergence = (
        los_truth_label is not None
        and los_cognition_label is not None
        and los_truth_label != los_cognition_label
    )

    per_entity = pose_block.get("per_entity") if pose_block else []
    if not isinstance(per_entity, list):
        per_entity = []

    return {
        "run_id": run["run_id"],
        "fidelity_attestation_status": _attestation_status(truth, pose_block, coupling_on),
        "los_truth_label": los_truth_label if coupling_on else None,
        "los_cognition_label": los_cognition_label,
        "visibility_truth_ref": (
            _record(truth.get("visibility_truth")).get("ref") if coupling_on and truth else None
        ),
        "dome_truth_ref": (
            _record(truth.get("dome_truth")).get("sensor_id") if coupling_on and truth else None
        ),
        "pose_truth_drift_m": (
            _max_numeric([e.get("pose_truth_drift_m") for e in per_entity if isinstance(e, dict)])
            if coupling_on
            else None
        ),
        "agl_truth_m": (
            _max_numeric([e.get("sim_agl_m") for e in per_entity if isinstance(e, dict)])
            if coupling_on
            else None
        ),
        "cognition_truth_divergence": cognition_truth_divergence,
    }


def build_fidelity_compare_pairs(per_run: list[dict[str, Any]]) -> list[dict[str, Any]]:
    sorted_rows = sorted(per_run, key=lambda r: r["run_id"])
    pairs: list[dict[str, Any]] = []

    for i, a in enumerate(sorted_rows):
        for b in sorted_rows[i + 1 :]:
            badges: list[dict[str, str]] = []

            if a.get("cognition_truth_divergence") or b.get("cognition_truth_divergence"):
                badges.append(
                    {"id": "cognition_truth_divergence", "label": "cognition_truth_divergence"}
                )

            if (
                a.get("los_truth_label") is not None
                and b.get("los_truth_label") is not None
                and a.get("los_truth_label") != b.get("los_truth_label")
            ):
                badges.append({"id": "los_truth_label_diff", "label": "los_truth_label_diff"})

            drift_a = a.get("pose_truth_drift_m")
            drift_b = b.get("pose_truth_drift_m")
            if (
                isinstance(drift_a, (int, float))
                and isinstance(drift_b, (int, float))
                and abs(drift_a - drift_b) > POSE_TRUTH_DRIFT_COMPARE_EPSILON_M
            ):
                badges.append({"id": "pose_truth_drift_delta", "label": "pose_truth_drift_delta"})

            status_asymmetric = (a.get("fidelity_attestation_status") == "available") != (
                b.get("fidelity_attestation_status") == "available"
            )
            if status_asymmetric:
                badges.append(
                    {
                        "id": "fidelity_attestation_asymmetric",
                        "label": "fidelity_attestation_asymmetric",
                    }
                )

            pairs.append({"run_id_a": a["run_id"], "run_id_b": b["run_id"], "badges": badges})

    return sorted(pairs, key=lambda p: (p["run_id_a"], p["run_id_b"]))


def rollup_fidelity(
    per_run: list[dict[str, Any]],
    manifest: dict[str, Any],
    *,
    repo_root: Path,
) -> dict[str, Any]:
    status_counts: dict[str, int] = {}
    cognition_truth_divergence_count = 0

    for row in per_run:
        status = str(row.get("fidelity_attestation_status", "unknown"))
        status_counts[status] = status_counts.get(status, 0) + 1
        if row.get("cognition_truth_divergence"):
            cognition_truth_divergence_count += 1

    fingerprint_groups: dict[str, dict[str, Any]] = {}
    runs = manifest.get("runs") or []
    for run in runs:
        if not isinstance(run, dict):
            continue
        spec_fp = str(run.get("spec_fingerprint") or "unknown")
        truth = _read_truth_snapshot(run, repo_root)
        ctx = _record(run.get("fidelity_context"))
        truth_fp = compute_truth_fingerprint(ctx, truth) or "unknown"
        group_key = f"{spec_fp}::{truth_fp}"
        if group_key not in fingerprint_groups:
            fingerprint_groups[group_key] = {
                "spec_fingerprint": spec_fp,
                "truth_fingerprint": truth_fp,
                "run_count": 0,
            }
        fingerprint_groups[group_key]["run_count"] += 1

    coupling_flag = any(
        _record(r.get("fidelity_context")).get("enable_fidelity_coupling")
        for r in runs
        if isinstance(r, dict)
    )

    truth_fingerprints = sorted(
        fingerprint_groups.values(),
        key=lambda g: (g["spec_fingerprint"], g["truth_fingerprint"]),
    )
    for entry in truth_fingerprints:
        entry["coupling_flag"] = coupling_flag

    return {
        "attestation_rollup": {"status_counts": status_counts},
        "divergence_rollup": {"cognition_truth_divergence_count": cognition_truth_divergence_count},
        "repeatability_truth_rollup": {"truth_fingerprints": truth_fingerprints},
    }


def _spec_fingerprint_from_manifest(manifest: dict[str, Any]) -> str | None:
    for run in manifest.get("runs") or []:
        if isinstance(run, dict) and run.get("spec_fingerprint"):
            return str(run["spec_fingerprint"])
    return None


def _coupling_required(manifest: dict[str, Any], spec: dict[str, Any] | None) -> bool:
    if spec and spec.get("enable_fidelity_coupling"):
        return True
    for run in manifest.get("runs") or []:
        if isinstance(run, dict) and _record(run.get("fidelity_context")).get(
            "enable_fidelity_coupling"
        ):
            return True
    return False


def derive_fidelity_metrics(
    manifest: dict[str, Any],
    f5_report: dict[str, Any] | None = None,
    *,
    spec: dict[str, Any] | None = None,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    del f5_report  # optional alignment only; unused in P2 derive body
    if manifest.get("schema") != "rt_experiment_manifest_v1":
        raise ValueError("manifest must be rt_experiment_manifest_v1")

    root = repo_root or _REPO_ROOT
    sorted_runs = sorted(
        [r for r in manifest.get("runs") or [] if isinstance(r, dict)],
        key=lambda r: str(r.get("run_id", "")),
    )

    per_run_fidelity = [per_run_fidelity_from_run(run, root) for run in sorted_runs]
    per_run_fidelity.sort(key=lambda r: r["run_id"])

    manifest_for_rollup = {**manifest, "runs": sorted_runs}

    return {
        "schema": "rt_experiment_fidelity_metrics_report_v1",
        "experiment_id": manifest.get("experiment_id") or "unknown",
        "governance_banner": FIDELITY_METRICS_GOVERNANCE_BANNER,
        "spec_fingerprint": _spec_fingerprint_from_manifest(manifest_for_rollup),
        "coupling_required": _coupling_required(manifest_for_rollup, spec),
        "per_run_fidelity": per_run_fidelity,
        "compare_pairs_fidelity": build_fidelity_compare_pairs(per_run_fidelity),
        "rollup_fidelity": rollup_fidelity(per_run_fidelity, manifest_for_rollup, repo_root=root),
    }


def _load_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Derive RT experiment fidelity metrics report")
    parser.add_argument("--manifest", required=True, type=Path, help="rt_experiment_manifest_v1 JSON")
    parser.add_argument(
        "--metrics",
        type=Path,
        help="optional rt_experiment_metrics_report_v1 (reserved for pair alignment)",
    )
    parser.add_argument("--spec", type=Path, help="optional rt_experiment_spec_v1")
    parser.add_argument("--repo-root", type=Path, default=_REPO_ROOT)
    parser.add_argument("--out", type=Path, help="write report JSON (adds derived_at_utc)")
    args = parser.parse_args(argv)

    manifest = _load_json(args.manifest)
    f5_report = _load_json(args.metrics) if args.metrics else None
    spec = _load_json(args.spec) if args.spec else None

    report = derive_fidelity_metrics(manifest, f5_report, spec=spec, repo_root=args.repo_root)
    report["derived_at_utc"] = datetime.now(timezone.utc).replace(microsecond=0).isoformat()

    text = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
