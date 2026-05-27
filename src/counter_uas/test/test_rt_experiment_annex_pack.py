"""Tests for PLAT-RT-F3 rt_experiment_annex_pack.py."""

from __future__ import annotations

import json
from pathlib import Path

from scripts.rt import rt_experiment_annex_pack as pack


def test_pack_annexes_from_staging(tmp_path: Path) -> None:
    staging = tmp_path / "runs/rt_sandbox/captures/cap-1"
    staging.mkdir(parents=True)
    annex = {
        "schema": "rt_tactical_capture_annex_v1",
        "authority_label": "replay_boundary_scoped",
        "governance_banner": "TACTICAL CAPTURE ANNEX — explanatory sandbox record; not SA replay authority",
        "final_tactical_mode": "manual",
        "mode_switches": [],
    }
    (staging / "tactical_annex.json").write_text(json.dumps(annex), encoding="utf-8")

    manifest = {
        "schema": "rt_experiment_manifest_v1",
        "experiment_id": "exp-pack",
        "runs": [
            {
                "run_id": "r1",
                "label": "one",
                "session_id": "s1",
                "recorded_at_utc": "2026-05-26T12:00:00+00:00",
                "capture_candidate_id": "cap-1",
                "capture_staging_ref": "runs/rt_sandbox/captures/cap-1",
                "snapshot": {},
            }
        ],
    }
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    bundle = pack.pack_annexes(manifest, repo_root=tmp_path)
    assert bundle["schema"] == "rt_experiment_annex_bundle_v1"
    assert len(bundle["entries"]) == 1
    assert bundle["entries"][0]["annex"]["final_tactical_mode"] == "manual"

    out = tmp_path / "bundle.json"
    code = pack.main(["--manifest", str(manifest_path), "--repo-root", str(tmp_path), "--out", str(out)])
    assert code == 0
    written = json.loads(out.read_text(encoding="utf-8"))
    assert written["entries"][0]["run_id"] == "r1"
