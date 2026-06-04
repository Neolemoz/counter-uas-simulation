"""rt_intelligence_advisory_transport_v1 telemetry tests."""

from __future__ import annotations

import json
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.governance import GovernanceConfig  # noqa: E402
from rt_sandbox.rt_intelligence_advisory_engine import GOVERNANCE_BANNER  # noqa: E402
from rt_sandbox.rt_intelligence_advisory_transport import (  # noqa: E402
    TRANSPORT_AUTHORITY,
    TRANSPORT_SCHEMA,
    TRANSPORT_SOURCE,
    build_intelligence_advisory_transport,
)
from rt_sandbox.telemetry_bridge import resolve_channel_payload  # noqa: E402
from rt_sandbox.telemetry_subscriptions import (  # noqa: E402
    MAX_CHANNELS_PER_SUBSCRIPTION,
    TELEMETRY_CHANNELS,
    TelemetrySubscriptionStore,
)

_FIXTURES = _REPO / "fixtures" / "rt_intelligence_advisory"


def _load(name: str) -> dict:
    data = json.loads((_FIXTURES / name).read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _session(session_id: str, advisory_input: dict | None = None) -> SimpleNamespace:
    session = SimpleNamespace(
        session_id=session_id,
        rt_intelligence_advisory_input=advisory_input,
        assignment_mutation_count=0,
        command_mutation_count=0,
    )
    return session


def _assert_transport_governance(payload: dict) -> None:
    assert payload["schema"] == TRANSPORT_SCHEMA
    assert payload["source"] == TRANSPORT_SOURCE
    assert payload["authority"] == TRANSPORT_AUTHORITY
    assert payload["governance_banner"] == GOVERNANCE_BANNER
    assert "authority_label" not in payload


def test_transport_fixture_parity() -> None:
    for name in (
        "transport_single_attacker_v1.json",
        "transport_multiple_attackers_v1.json",
        "transport_no_solution_v1.json",
    ):
        expected = _load(name)
        input_name = name.replace("transport_", "").replace("_v1.json", "_input_v1.json")
        session = _session(expected["session_id"], _load(input_name))
        assert build_intelligence_advisory_transport(session) == expected


def test_payload_generation_and_governance_fields() -> None:
    payload = build_intelligence_advisory_transport(
        _session("session-single", _load("single_attacker_input_v1.json"))
    )
    _assert_transport_governance(payload)
    assert payload["stale"] is False
    assert payload["stale_reason"] is None
    assert len(payload["advisories"]) == 1
    assert payload["advisories"][0]["identity"]["attacker_id"] == "attacker-alpha"


def test_multiple_advisories_are_deterministic() -> None:
    session = _session("session-multiple", _load("multiple_attackers_input_v1.json"))
    first = build_intelligence_advisory_transport(session)
    second = build_intelligence_advisory_transport(session)
    assert first == second
    assert [a["identity"]["attacker_id"] for a in first["advisories"]] == [
        "attacker-b",
        "attacker-a",
        "attacker-c",
    ]


def test_no_solution_transport() -> None:
    payload = build_intelligence_advisory_transport(
        _session("session-no-solution", _load("no_solution_input_v1.json"))
    )
    advisory = payload["advisories"][0]
    assert advisory["recommended_defender"]["defender_id"] is None
    assert advisory["recommended_defender"]["feasibility"] == {
        "feasible": False,
        "reason": "no_solution",
    }
    assert "no_solution" in advisory["reasoning"]["reason_codes"]


def test_empty_advisory_list() -> None:
    data = deepcopy(_load("single_attacker_input_v1.json"))
    data["session_id"] = "session-empty"
    data["advisory_utc"] = "2026-06-05T10:03:00Z"
    data["attackers"] = []
    payload = build_intelligence_advisory_transport(_session("session-empty", data))
    assert payload["stale"] is False
    assert payload["stale_reason"] is None
    assert payload["advisories"] == []


def test_stale_behavior_for_missing_and_explicit_stale_inputs() -> None:
    missing = build_intelligence_advisory_transport(
        _session("session-missing", None),
        now_utc="2026-06-05T10:04:00Z",
    )
    assert missing["stale"] is True
    assert missing["stale_reason"] == "input_unavailable"
    assert missing["advisories"] == []

    explicit = deepcopy(_load("single_attacker_input_v1.json"))
    explicit["stale"] = True
    explicit["stale_reason"] = "source_stale"
    stale = build_intelligence_advisory_transport(_session("session-single", explicit))
    assert stale["stale"] is True
    assert stale["stale_reason"] == "source_stale"
    assert stale["advisories"] == []


def test_telemetry_channel_resolves_without_assignment_or_command_mutation() -> None:
    session = _session("session-single", _load("single_attacker_input_v1.json"))
    payload = resolve_channel_payload(
        session,
        "intelligence_advisory",
        GovernanceConfig(),
    )
    assert payload is not None
    _assert_transport_governance(payload)
    assert payload["advisories"][0]["identity"]["attacker_id"] == "attacker-alpha"
    assert session.assignment_mutation_count == 0
    assert session.command_mutation_count == 0


def test_telemetry_allow_list_accepts_advisory_channel() -> None:
    assert "intelligence_advisory" in TELEMETRY_CHANNELS
    assert MAX_CHANNELS_PER_SUBSCRIPTION == 8
    store = TelemetrySubscriptionStore()
    sub_id, err = store.subscribe("session-single", ["intelligence_advisory"])
    assert err is None
    assert sub_id is not None


def test_invalid_input_is_stale_and_read_only() -> None:
    session = _session("session-invalid")
    session.rt_intelligence_advisory_input = ["not", "a", "dict"]
    payload = build_intelligence_advisory_transport(
        session,
        now_utc="2026-06-05T10:05:00Z",
    )
    assert payload["stale"] is True
    assert payload["stale_reason"] == "input_invalid"
    assert payload["advisories"] == []
    assert session.assignment_mutation_count == 0
    assert session.command_mutation_count == 0
