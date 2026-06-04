"""Read-only RT intelligence advisory telemetry transport."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from rt_sandbox.rt_intelligence_advisory_engine import (
    GOVERNANCE_BANNER as ADVISORY_GOVERNANCE_BANNER,
    build_intelligence_advisories,
)

TRANSPORT_SCHEMA = "rt_intelligence_advisory_transport_v1"
TRANSPORT_SOURCE = "rt_intelligence_advisory_engine"
TRANSPORT_AUTHORITY = "recommendation_only"
INPUT_ATTR = "rt_intelligence_advisory_input"


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _session_id(session: Any, data: dict[str, Any] | None = None) -> str:
    if data is not None and data.get("session_id"):
        return str(data["session_id"])
    return str(getattr(session, "session_id", ""))


def _advisory_utc(data: dict[str, Any] | None, now_utc: str | None) -> str:
    if data is not None and data.get("advisory_utc"):
        return str(data["advisory_utc"])
    return str(now_utc or _utc_now())


def _stale_reason(data: Any, explicit_stale_reason: str | None) -> str | None:
    if explicit_stale_reason:
        return explicit_stale_reason
    if data is None:
        return "input_unavailable"
    if not isinstance(data, dict):
        return "input_invalid"
    if data.get("stale") is True:
        return str(data.get("stale_reason") or "source_stale")
    return None


def build_intelligence_advisory_transport(
    session: Any,
    *,
    advisory_input: dict[str, Any] | None = None,
    refresh_reason: str = "snapshot",
    stale_reason: str | None = None,
    now_utc: str | None = None,
) -> dict[str, Any]:
    """Build rt_intelligence_advisory_transport_v1 without side effects."""
    raw_input: Any = advisory_input
    if raw_input is None:
        raw_input = getattr(session, INPUT_ATTR, None)

    reason = _stale_reason(raw_input, stale_reason)
    data = raw_input if isinstance(raw_input, dict) else None
    advisories = [] if reason else build_intelligence_advisories(data or {})
    advisory_utc = _advisory_utc(data, now_utc)

    return {
        "schema": TRANSPORT_SCHEMA,
        "session_id": _session_id(session, data),
        "advisory_utc": advisory_utc,
        "source": TRANSPORT_SOURCE,
        "authority": TRANSPORT_AUTHORITY,
        "governance_banner": ADVISORY_GOVERNANCE_BANNER,
        "refresh_reason": refresh_reason,
        "stale": reason is not None,
        "stale_reason": reason,
        "advisories": advisories,
    }
