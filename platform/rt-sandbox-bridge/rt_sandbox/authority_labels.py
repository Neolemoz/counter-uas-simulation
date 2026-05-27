"""Authority labels for RT telemetry and capture surfaces (PLAT-RT-R1a).

See docs/evaluation/rt_authority_model_v1.md.
"""

from __future__ import annotations

from typing import Any

from rt_sandbox.governance import GOVERNANCE_BANNER

AUTHORITY_COMMAND = "command_authoritative"
AUTHORITY_EXPLANATORY_SYNC = "explanatory_sync"
AUTHORITY_EXPLANATORY_TELEMETRY = "explanatory_telemetry"
AUTHORITY_REPLAY_BOUNDARY = "replay_boundary_scoped"
AUTHORITY_TACTICAL_RECOMMENDATION = "tactical_recommendation_explanatory"
AUTHORITY_USER_APPROVAL = "user_approval_authoritative"
AUTHORITY_TACTICAL_CONTROLLER = "tactical_controller_authoritative"
AUTHORITY_TRUTH_ATTESTED = "truth_attested"

AUTONOMOUS_LOOP_BANNER = (
    "AUTONOMOUS LOOP — user may revert to Manual; not replay authority"
)

SOURCE_BRIDGE_REGISTRY = "bridge_registry"
SOURCE_BRIDGE_SESSION = "bridge_session"
SOURCE_ADAPTER_FEEDBACK = "adapter_feedback"
SOURCE_ADAPTER_TELEMETRY = "adapter_telemetry"
SOURCE_TACTICAL_CONTROLLER = "rt_tactical_controller"


def authority_model_legend() -> dict[str, str]:
    return {
        "command_authoritative": "Bridge EntityRegistry command truth",
        "explanatory_sync": "PoseSyncMirror drift/stale evidence only",
        "explanatory_telemetry": "TelemetryMirror read model; not replay authority",
        "replay_boundary_scoped": "RT staging artifact; SA import requires approval",
        "truth_attested": "Sim-scoped attestation when fidelity coupling on; not registry authority",
    }


def enrich_channel_payload(
    payload: dict[str, Any],
    *,
    source: str,
    authority_label: str,
    governance_banner: str | None = GOVERNANCE_BANNER,
) -> dict[str, Any]:
    out = dict(payload)
    out["source"] = source
    out["authority_label"] = authority_label
    if governance_banner and "governance_banner" not in out:
        out["governance_banner"] = governance_banner
    return out
