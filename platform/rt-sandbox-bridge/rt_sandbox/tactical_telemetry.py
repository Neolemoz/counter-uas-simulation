"""Tactical telemetry payload builders (PLAT-RT-TAC2 / TAC3 / TAC4)."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Callable

from rt_sandbox.authority_labels import (
    AUTHORITY_COMMAND,
    AUTHORITY_TACTICAL_CONTROLLER,
    AUTHORITY_TACTICAL_RECOMMENDATION,
    AUTHORITY_USER_APPROVAL,
    AUTONOMOUS_LOOP_BANNER,
    SOURCE_BRIDGE_SESSION,
    SOURCE_TACTICAL_CONTROLLER,
    enrich_channel_payload,
)
from rt_sandbox.session_record import SessionRecord
from rt_sandbox.tactical_controller import TacticalController
from rt_sandbox.tactical_recommendation import (
    RECOMMENDATION_GOVERNANCE_BANNER,
    cleared_recommendation,
)
from rt_sandbox.tactical_state import TACTICAL_MODE_AUTONOMOUS, TACTICAL_TELEMETRY_BANNER


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _tactical_state_authority(
    session: SessionRecord,
    *,
    user_approval: bool = False,
) -> str:
    if user_approval:
        return AUTHORITY_USER_APPROVAL
    tactical: TacticalController | None = getattr(session, "tactical", None)
    if tactical is not None and tactical.state.mode == TACTICAL_MODE_AUTONOMOUS:
        return AUTHORITY_TACTICAL_CONTROLLER
    return AUTHORITY_COMMAND


def _tactical_state_banner(session: SessionRecord) -> str:
    tactical: TacticalController | None = getattr(session, "tactical", None)
    if tactical is not None and tactical.state.mode == TACTICAL_MODE_AUTONOMOUS:
        return AUTONOMOUS_LOOP_BANNER
    return TACTICAL_TELEMETRY_BANNER


def build_tactical_state_payload(
    session: SessionRecord,
    *,
    authority_label: str | None = None,
    user_approval: bool = False,
) -> dict[str, Any]:
    label = authority_label or _tactical_state_authority(
        session, user_approval=user_approval
    )
    tactical: TacticalController | None = getattr(session, "tactical", None)
    if tactical is None:
        body: dict[str, Any] = {
            "schema": "rt_tactical_state_v1",
            "session_id": session.session_id,
            "tactical_mode": "manual",
        }
    else:
        body = tactical.snapshot_dict()
    body["sampled_at_utc"] = _utc_now()
    return enrich_channel_payload(
        body,
        source=SOURCE_BRIDGE_SESSION,
        authority_label=label,
        governance_banner=_tactical_state_banner(session),
    )


def build_tactical_recommendation_payload(session: SessionRecord) -> dict[str, Any]:
    tactical: TacticalController | None = getattr(session, "tactical", None)
    if tactical is None or tactical.state.pending_recommendation is None:
        body = cleared_recommendation(session.session_id).to_dict()
    else:
        body = dict(tactical.state.pending_recommendation.to_dict())
    body["schema"] = "rt_tactical_recommendation_v1"
    body["session_id"] = session.session_id
    body["tactical_health"] = (
        dict(tactical.state.tactical_health)
        if tactical is not None
        else {"feasible": False, "summary": "no_selection", "stale": False}
    )
    body["sampled_at_utc"] = _utc_now()
    return enrich_channel_payload(
        body,
        source=SOURCE_TACTICAL_CONTROLLER,
        authority_label=AUTHORITY_TACTICAL_RECOMMENDATION,
        governance_banner=RECOMMENDATION_GOVERNANCE_BANNER,
    )


def publish_tactical_state(
    session: SessionRecord,
    publish_channel: Any,
    *,
    authority_label: str | None = None,
) -> None:
    publish_channel("tactical_state")


def publish_tactical_recommendation(
    session: SessionRecord,
    publish_channel: Callable[[str], None],
) -> None:
    publish_channel("tactical_recommendation")


def tactical_state_snapshot_for_response(
    session: SessionRecord,
    *,
    user_approval: bool = False,
) -> dict[str, Any]:
    label = _tactical_state_authority(session, user_approval=user_approval)
    tactical: TacticalController | None = getattr(session, "tactical", None)
    if tactical is None:
        return build_tactical_state_payload(
            session, authority_label=label, user_approval=user_approval
        )
    body = tactical.snapshot_dict()
    body["sampled_at_utc"] = _utc_now()
    return enrich_channel_payload(
        body,
        source=SOURCE_BRIDGE_SESSION,
        authority_label=label,
        governance_banner=_tactical_state_banner(session),
    )


def tactical_recommendation_snapshot_for_response(
    session: SessionRecord,
) -> dict[str, Any]:
    return build_tactical_recommendation_payload(session)
