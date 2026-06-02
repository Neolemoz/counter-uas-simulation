from __future__ import annotations

import importlib.util
import json
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_GOLDEN = _REPO_ROOT / "fixtures" / "rt_sandbox" / "rt_layout_scenario_golden_v1.json"


def _load_mod():  # noqa: ANN201
    path = _REPO_ROOT / "scripts" / "evaluation" / "rt_layout_mc_profile.py"
    assert path.is_file(), f"missing {path}"
    spec = importlib.util.spec_from_file_location("rt_layout_mc_profile", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _golden_layout() -> dict:
    return json.loads(_GOLDEN.read_text(encoding="utf-8"))


def test_rt_layout_schema_validation_accepts_golden_fixture() -> None:
    mod = _load_mod()
    result = mod.validate_layout(_golden_layout())

    assert result["ok"] is True
    assert result["issues"] == []


def test_rt_layout_translator_maps_single_drone_and_three_interceptors() -> None:
    mod = _load_mod()
    profile = mod.translate_layout_to_profile(_golden_layout())

    assert profile["schema_version"] == "rt_layout_mc_profile_v1"
    assert profile["source_layout_id"] == "rt_layout_mc_golden"
    assert profile["geometry_id"].startswith("rt_layout:sha256:")
    assert profile["scenario_suggestion"] == "single"
    assert profile["launch_args_fields"] == {
        "target_start_x_m": "-1500",
        "target_start_y_m": "0",
        "target_start_z_m": "300",
        "interceptor_ic_layout": "custom:-5,0,4,-4,-4,5",
    }
    assert "target_start_x_m:=-1500" in profile["launch_args"]
    assert "interceptor_ic_layout:=custom:-5,0,4,-4,-4,5" in profile["launch_args"]
    assert profile["entity_counts"]["radar"] == 1
    assert profile["entity_counts"]["waypoint_marker"] == 1
    assert any("radar entities are retained as metadata" in w for w in profile["warnings"])


def test_rt_layout_translator_warns_on_unsupported_multi_drone() -> None:
    mod = _load_mod()
    layout = _golden_layout()
    layout["entities"].append(
        {
            "entity_type": "drone",
            "pose": {"x": -1200.0, "y": 250.0, "z": 250.0, "yaw_deg": 0.0},
        }
    )

    profile = mod.translate_layout_to_profile(layout)

    assert profile["scenario_suggestion"] == "multi"
    assert "target_start_x_m" not in profile["launch_args_fields"]
    assert any("multiple drone entities" in w for w in profile["warnings"])
    assert any(row["field"] == "entities[drone]" for row in profile["unsupported_fields"])


def test_rt_layout_geometry_fingerprint_ignores_metadata_notes() -> None:
    mod = _load_mod()
    layout_a = _golden_layout()
    layout_b = _golden_layout()
    layout_b["notes"] = "changed explanatory note"
    layout_b["created_utc"] = "2026-06-02T12:00:00Z"

    assert mod.geometry_fingerprint(layout_a) == mod.geometry_fingerprint(layout_b)
