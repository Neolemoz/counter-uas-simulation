#!/usr/bin/env python3
"""Maintainer-only RT experiment batch runner (PLAT-RT-X1).

Sequences start_session → optional template → dwell → stop_session → capture_session.
Updates rt_experiment_manifest_v1 under runs/rt_sandbox/experiments/.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
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

from scripts.rt.rt_bridge_client import send_command  # noqa: E402

EXPERIMENT_GOVERNANCE_BANNER = (
    "RT EXPERIMENT — explanatory compare only; not operational authority"
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _load_spec(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix in {".yaml", ".yml"}:
        if yaml is None:
            raise RuntimeError("PyYAML required for YAML batch specs")
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict) or data.get("schema") != "rt_experiment_batch_v1":
        raise ValueError("spec must be rt_experiment_batch_v1")
    return data


def _annex_summary(staging_dir: Path) -> dict[str, Any] | None:
    annex_path = staging_dir / "tactical_annex.json"
    if not annex_path.is_file():
        return None
    data = json.loads(annex_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return None
    return {
        "final_tactical_mode": data.get("final_tactical_mode"),
        "selected_id": data.get("selected_id"),
        "assigned_target": data.get("assigned_target"),
        "timeline_counts": {
            "mode_switches": len(data.get("mode_switches") or []),
            "assignment_timeline": len(data.get("assignment_timeline") or []),
            "pause_resume_transitions": len(data.get("pause_resume_transitions") or []),
            "recommendation_timeline": len(data.get("recommendation_timeline") or []),
        },
    }


def _capture_staging_ref(repo_root: Path, capture_id: str) -> str:
    return f"runs/rt_sandbox/captures/{capture_id}"


def _load_manifest(path: Path, experiment_id: str) -> dict[str, Any]:
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return {
        "schema": "rt_experiment_manifest_v1",
        "experiment_id": experiment_id,
        "created_at_utc": _utc_now(),
        "governance_banner": EXPERIMENT_GOVERNANCE_BANNER,
        "runs": [],
    }


def _write_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _audit(audit_path: Path, event: dict[str, Any]) -> None:
    audit_path.parent.mkdir(parents=True, exist_ok=True)
    with audit_path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps({"t_utc": _utc_now(), **event}, sort_keys=True) + "\n")


def run_batch(
    *,
    spec: dict[str, Any],
    repo_root: Path,
    manifest_path: Path,
    command_url: str,
    dry_run: bool,
    audit_path: Path,
) -> int:
    experiment_id = str(spec["experiment_id"])
    default_dwell = float(spec.get("default_dwell_s") or 2.0)
    manifest = _load_manifest(manifest_path, experiment_id)
    runs_spec = spec.get("runs") or []
    if not isinstance(runs_spec, list) or not runs_spec:
        print("batch spec has no runs", file=sys.stderr)
        return 2

    for item in runs_spec:
        run_id = str(item["run_id"])
        label = str(item.get("label") or run_id)
        dwell_s = float(item.get("dwell_s") or default_dwell)
        template_id = item.get("template_id")

        _audit(
            audit_path,
            {"step": "run_start", "run_id": run_id, "dry_run": dry_run},
        )

        if dry_run:
            print(f"[dry-run] would run {run_id} dwell={dwell_s}s template={template_id}")
            continue

        start = send_command("start_session", url=command_url, issued_by="rt_experiment_batch")
        if not start.get("ok"):
            print(f"start_session failed: {start}", file=sys.stderr)
            return 1
        session_id = str(start.get("session_id") or "")
        _audit(audit_path, {"step": "start_session", "run_id": run_id, "session_id": session_id})

        if template_id:
            tmpl = send_command(
                "apply_runtime_template",
                url=command_url,
                session_id=session_id,
                payload={"template_id": str(template_id)},
                issued_by="rt_experiment_batch",
            )
            _audit(audit_path, {"step": "apply_runtime_template", "result": tmpl.get("ok")})
            if not tmpl.get("ok"):
                print(f"apply_runtime_template failed: {tmpl}", file=sys.stderr)
                return 1

        time.sleep(max(0.0, dwell_s))

        stop = send_command(
            "stop_session",
            url=command_url,
            session_id=session_id,
            issued_by="rt_experiment_batch",
        )
        _audit(audit_path, {"step": "stop_session", "result": stop.get("ok")})
        if not stop.get("ok"):
            print(f"stop_session failed: {stop}", file=sys.stderr)
            return 1

        capture = send_command(
            "capture_session",
            url=command_url,
            session_id=session_id,
            issued_by="rt_experiment_batch",
        )
        _audit(audit_path, {"step": "capture_session", "result": capture.get("ok")})
        if not capture.get("ok"):
            print(f"capture_session failed: {capture}", file=sys.stderr)
            return 1

        detail = capture.get("detail") if isinstance(capture.get("detail"), dict) else {}
        capture_id = str(detail.get("capture_candidate_id") or capture.get("capture_candidate_id") or "")
        staging_ref = _capture_staging_ref(repo_root, capture_id) if capture_id else None
        staging_dir = repo_root / staging_ref if staging_ref else None
        annex = _annex_summary(staging_dir) if staging_dir and staging_dir.is_dir() else None

        run_record = {
            "run_id": run_id,
            "label": label,
            "session_id": session_id,
            "recorded_at_utc": _utc_now(),
            "capture_candidate_id": capture_id or None,
            "capture_staging_ref": staging_ref,
            "snapshot": {
                "tactical_state": capture.get("tactical_state"),
                "world_summary": capture.get("world_summary"),
                "lifecycle_state": {"state": "captured"},
            },
            "tactical_annex_summary": annex,
        }
        for key in (
            "experiment_class",
            "spec_fingerprint",
            "matrix_coords",
            "repeat_index",
            "repeat_group_id",
            "terrain_profile_ref",
        ):
            if key in item:
                run_record[key] = item[key]
        if item.get("f4_layer_preset"):
            run_record["visibility_context"] = {
                "f4_layer_preset": item["f4_layer_preset"],
            }
        manifest["runs"] = [r for r in manifest.get("runs", []) if r.get("run_id") != run_id]
        manifest["runs"].append(run_record)
        _write_manifest(manifest_path, manifest)
        print(f"OK {run_id} capture={capture_id} manifest={manifest_path}")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RT experiment batch runner")
    parser.add_argument("--spec", required=True, help="rt_experiment_batch_v1 YAML/JSON")
    parser.add_argument("--repo-root", default=str(_REPO_ROOT))
    parser.add_argument(
        "--manifest-out",
        default=None,
        help="manifest path (default runs/rt_sandbox/experiments/<id>/manifest.json)",
    )
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:18765/v1/command",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    spec_path = Path(args.spec)
    if not spec_path.is_file():
        print(f"spec not found: {spec_path}", file=sys.stderr)
        return 2
    spec = _load_spec(spec_path)
    repo_root = Path(args.repo_root).resolve()
    experiment_id = str(spec["experiment_id"])
    manifest_path = (
        Path(args.manifest_out)
        if args.manifest_out
        else repo_root / "runs" / "rt_sandbox" / "experiments" / experiment_id / "manifest.json"
    )
    audit_path = manifest_path.parent / "experiment_batch_audit.jsonl"

    return run_batch(
        spec=spec,
        repo_root=repo_root,
        manifest_path=manifest_path,
        command_url=args.url,
        dry_run=args.dry_run,
        audit_path=audit_path,
    )


if __name__ == "__main__":
    raise SystemExit(main())
