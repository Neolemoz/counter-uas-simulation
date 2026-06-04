"""Pure rt_intelligence_advisory_v1 engine tests."""

from __future__ import annotations

import json
import sys
from copy import deepcopy
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.rt_intelligence_advisory_engine import (  # noqa: E402
    AUTHORITY,
    GOVERNANCE_BANNER,
    build_intelligence_advisories,
    build_intelligence_advisory,
    compute_threat_score,
    derive_heuristic_confidence,
    derive_reason_codes,
    rank_attackers,
    rank_defenders,
)

_FIXTURES = _REPO / "fixtures" / "rt_intelligence_advisory"


def _load(name: str) -> dict | list:
    return json.loads((_FIXTURES / name).read_text(encoding="utf-8"))


def _input(name: str) -> dict:
    data = _load(f"{name}_input_v1.json")
    assert isinstance(data, dict)
    return data


def _expected(name: str) -> list[dict]:
    data = _load(f"{name}_expected_v1.json")
    assert isinstance(data, list)
    return data


def test_fixture_parity() -> None:
    for name in (
        "single_attacker",
        "multiple_attackers",
        "critical_descending",
        "no_solution",
        "tti_tie_break",
    ):
        assert build_intelligence_advisories(_input(name)) == _expected(name)


def test_build_single_advisory() -> None:
    advisory = build_intelligence_advisory(_input("single_attacker"), "attacker-alpha")
    assert advisory["identity"]["attacker_id"] == "attacker-alpha"
    assert advisory["recommended_defender"]["defender_id"] == "defender-a"


def test_threat_score_bounds() -> None:
    data = _input("critical_descending")
    attacker = data["attackers"][0]
    constants = data["constants"]
    score, components = compute_threat_score(attacker, 8.0, constants)
    assert score is not None
    assert 0.0 <= score <= 100.0
    for component in components.values():
        normalized = component.get("normalized")
        if normalized is not None:
            assert 0.0 <= normalized <= 1.0


def test_threat_ranking_and_tie_breaks() -> None:
    ranks = rank_attackers(_input("multiple_attackers"))
    assert ranks == {
        "attacker-b": 1,
        "attacker-a": 2,
        "attacker-c": 3,
    }


def test_defender_ranking_feasible_tti_and_tie_break() -> None:
    ranked = rank_defenders(_input("tti_tie_break"), "attacker-tie")
    assert [row["defender_id"] for row in ranked] == ["defender-a", "defender-b"]
    assert ranked[0]["rank"] == 1
    assert ranked[0]["tti_s"] == 20.0
    assert "tti_tie_break" in ranked[0]["reason_codes"]


def test_no_solution_case() -> None:
    advisory = build_intelligence_advisory(_input("no_solution"), "attacker-no-solution")
    assert advisory["recommended_defender"] == {
        "defender_id": None,
        "feasibility": {"feasible": False, "reason": "no_solution"},
        "tti_s": None,
    }
    assert "no_solution" in advisory["reasoning"]["reason_codes"]
    assert advisory["confidence"]["heuristic_confidence"]["level"] == "low"


def test_reason_code_mapping() -> None:
    data = _input("critical_descending")
    ranked = rank_defenders(data, "attacker-critical")
    codes = derive_reason_codes(data["attackers"][0], ranked, data["constants"])
    assert "critical_target" in codes
    assert "inside_warning_ring" in codes
    assert "descending_fast" in codes
    assert "shortest_tti" in codes
    assert "only_feasible" in codes


def test_confidence_levels() -> None:
    high = build_intelligence_advisory(_input("single_attacker"), "attacker-alpha")
    assert high["confidence"]["heuristic_confidence"]["level"] == "high"

    medium_input = deepcopy(_input("single_attacker"))
    medium_input["attackers"][0]["descent_rate_mps"] = None
    medium = build_intelligence_advisory(medium_input, "attacker-alpha")
    assert medium["confidence"]["heuristic_confidence"]["level"] == "medium"

    low = build_intelligence_advisory(_input("no_solution"), "attacker-no-solution")
    assert low["confidence"]["heuristic_confidence"]["level"] == "low"


def test_payload_governance_and_no_extra_authority_fields() -> None:
    advisory = build_intelligence_advisory(_input("single_attacker"), "attacker-alpha")
    assert advisory["schema"] == "rt_intelligence_advisory_v1"
    assert advisory["governance"] == {
        "authority": AUTHORITY,
        "governance_banner": GOVERNANCE_BANNER,
    }
    assert set(advisory) == {
        "schema",
        "identity",
        "threat_evaluation",
        "recommended_defender",
        "defender_ranking",
        "reasoning",
        "confidence",
        "governance",
    }
