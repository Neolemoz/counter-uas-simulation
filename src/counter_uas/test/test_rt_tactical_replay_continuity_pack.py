"""PLAT-RT-SA3 — replay_sa_bundle RT tactical continuity embed tests."""

from __future__ import annotations

import json
import sys
import uuid
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
_EVAL = _REPO / "scripts" / "evaluation"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from replay_sa_bundle import lint_replay_sa_bundle, pack_bundle  # noqa: E402
from rt_tactical_replay_continuity import (  # noqa: E402
    load_rt_tactical_annex_from_capture_staging,
)


def _minimal_narrative() -> dict:
    return {
        "artifact_type": "replay_narrative_report",
        "narrative_schema_version": "replay_narrative_v1",
        "summary": {"run_id": "rt_sa3_test"},
        "events": [],
        "windows": [],
        "lineage": {"run_id": "rt_sa3_test", "log_path": "runs/rt_sandbox/captures/x/log.txt"},
    }


def _minimal_observability() -> dict:
    return {
        "artifact_type": "replay_observability",
        "lineage": {"run_id": "rt_sa3_test"},
    }


@pytest.fixture
def rt_staging(tmp_path: Path) -> Path:
    staging = tmp_path / "capture-staging"
    staging.mkdir()
    capture_id = str(uuid.uuid4())
    annex = {
        "schema": "rt_tactical_capture_annex_v1",
        "origin": "rt_sandbox_tactical_v1",
        "capture_candidate_id": capture_id,
        "ephemeral_session_ref": "sess-1",
        "final_tactical_mode": "assisted",
        "selected_id": "int-1",
        "assigned_target": "tgt-1",
        "selected_timeline": [],
        "assignment_timeline": [
            {
                "t_utc": "2026-05-26T10:00:00+00:00",
                "assigned_candidate_id": "int-1",
                "reason": "approved_recommendation",
                "authority_label": "replay_boundary_scoped",
            }
        ],
        "tti_timeline": [],
        "recommendation_timeline": [{"t_utc": "2026-05-26T09:59:00+00:00", "event": "issued"}],
        "mode_switches": [],
        "pause_resume_transitions": [],
        "assignment_lock_events": [],
        "target_switch_events": [],
        "authority_label": "replay_boundary_scoped",
        "governance_banner": "TACTICAL CAPTURE ANNEX — explanatory sandbox record; not SA replay authority",
    }
    (staging / "tactical_annex.json").write_text(json.dumps(annex), encoding="utf-8")
    (staging / "candidate.json").write_text(
        json.dumps(
            {
                "schema": "rt_capture_candidate_v1",
                "capture_candidate_id": capture_id,
                "session_id": "sess-1",
            }
        ),
        encoding="utf-8",
    )
    (staging / "provenance.json").write_text(
        json.dumps(
            {
                "schema": "rt_capture_provenance_v1",
                "source_artifact_refs": {"tactical_annex_ref": str(staging / "tactical_annex.json")},
            }
        ),
        encoding="utf-8",
    )
    return staging


def test_load_annex_from_staging(rt_staging: Path) -> None:
    annex = load_rt_tactical_annex_from_capture_staging(rt_staging)
    assert annex is not None
    assert annex["schema"] == "rt_tactical_capture_annex_v1"


def test_pack_embeds_continuity(tmp_path: Path, rt_staging: Path) -> None:
    narr_path = tmp_path / "narrative.json"
    obs_path = tmp_path / "obs.json"
    out_dir = tmp_path / "bundle"
    narr_path.write_text(json.dumps(_minimal_narrative()), encoding="utf-8")
    obs_path.write_text(json.dumps(_minimal_observability()), encoding="utf-8")

    bundle = pack_bundle(
        narrative_json=narr_path,
        observability_json=obs_path,
        rt_capture_staging_dir=rt_staging,
        out_dir=out_dir,
    )
    assert bundle.get("rt_tactical_replay_continuity") is not None
    assert bundle["rt_tactical_replay_continuity"]["continuity_available"] is True
    assert (out_dir / "tactical_annex.json").is_file()
    lint = lint_replay_sa_bundle(bundle)
    assert lint["ok"], lint["issues"]
