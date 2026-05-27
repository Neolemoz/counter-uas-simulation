"""Audit event_kind taxonomy (PLAT-RT-R1a).

See docs/evaluation/rt_audit_event_vocabulary_v1.md.
"""

from __future__ import annotations

from rt_sandbox.governance import ALLOWED_COMMANDS, RUNTIME_SUBCOMMANDS

EVENT_KIND_USER = "user_command"
EVENT_KIND_ADAPTER = "adapter"
EVENT_KIND_SYNC = "sync"
EVENT_KIND_TELEMETRY = "telemetry"
EVENT_KIND_LIFECYCLE = "lifecycle"
EVENT_KIND_CLEANUP = "cleanup"
EVENT_KIND_EXPORT = "export"
EVENT_KIND_CAPTURE = "capture"
EVENT_KIND_TACTICAL = "tactical"

# PLAT-RT-SA1 export-boundary handoff events (event_kind remains export in JSONL)
HANDOFF_EVENT_TYPES = frozenset(
    {
        "handoff_ready",
        "handoff_reviewed",
        "handoff_rejected",
        "handoff_import_deferred",
        "handoff_import_prepared",
        "handoff_import_committed",
    }
)

VALID_EVENT_KINDS = frozenset(
    {
        EVENT_KIND_USER,
        EVENT_KIND_ADAPTER,
        EVENT_KIND_SYNC,
        EVENT_KIND_TELEMETRY,
        EVENT_KIND_LIFECYCLE,
        EVENT_KIND_CLEANUP,
        EVENT_KIND_EXPORT,
        EVENT_KIND_CAPTURE,
        EVENT_KIND_TACTICAL,
    }
)

_COMMAND_TYPE_TO_KIND: dict[str, str] = {
    "adapter_attach": EVENT_KIND_ADAPTER,
    "adapter_detach": EVENT_KIND_ADAPTER,
    "adapter_teardown": EVENT_KIND_ADAPTER,
    "adapter_health": EVENT_KIND_ADAPTER,
    "adapter_feedback_lost": EVENT_KIND_ADAPTER,
    "mock_inject_drift": EVENT_KIND_ADAPTER,
    "sync_update": EVENT_KIND_SYNC,
    "sync_stale": EVENT_KIND_SYNC,
    "sync_mismatch": EVENT_KIND_SYNC,
    "sync_lag_observed": EVENT_KIND_SYNC,
    "sync_missing_feedback_entity": EVENT_KIND_SYNC,
    "gz_entity_spawned": EVENT_KIND_ADAPTER,
    "gz_entity_pose_applied": EVENT_KIND_ADAPTER,
    "gz_entity_removed": EVENT_KIND_ADAPTER,
    "adapter_live_poll_tick": EVENT_KIND_ADAPTER,
    "template_resync_requested": EVENT_KIND_SYNC,
    "template_resync_completed": EVENT_KIND_SYNC,
    "template_resync_skipped": EVENT_KIND_SYNC,
    "template_resync_stale": EVENT_KIND_SYNC,
    "telemetry_update": EVENT_KIND_TELEMETRY,
    "telemetry_stale": EVENT_KIND_TELEMETRY,
    "telemetry_feedback_lost": EVENT_KIND_TELEMETRY,
    "telemetry_cleanup": EVENT_KIND_TELEMETRY,
    "telemetry_buffer_trim": EVENT_KIND_TELEMETRY,
    "telemetry_snapshot": EVENT_KIND_TELEMETRY,
    "fidelity_truth_update": EVENT_KIND_TELEMETRY,
    "fidelity_truth_stale": EVENT_KIND_TELEMETRY,
    "fidelity_truth_mismatch": EVENT_KIND_SYNC,
    "entity_cleanup": EVENT_KIND_CLEANUP,
    "orphan_cleanup": EVENT_KIND_CLEANUP,
    "auto_cleanup": EVENT_KIND_LIFECYCLE,
    "bridge_ready_timeout": EVENT_KIND_LIFECYCLE,
    "max_session_duration": EVENT_KIND_LIFECYCLE,
    "runtime_crashed": EVENT_KIND_LIFECYCLE,
    "capture_pose_authority": EVENT_KIND_CAPTURE,
    "capture_pose_stale": EVENT_KIND_CAPTURE,
    "capture_pose_mismatch": EVENT_KIND_CAPTURE,
    "fidelity_capture_snapshot": EVENT_KIND_CAPTURE,
    "tactical_mode_changed": EVENT_KIND_TACTICAL,
    "tactical_candidate_selected": EVENT_KIND_TACTICAL,
    "tactical_assignment_committed": EVENT_KIND_TACTICAL,
    "tactical_assignment_cleared": EVENT_KIND_TACTICAL,
    "tactical_recommendation_issued": EVENT_KIND_TACTICAL,
    "tactical_recommendation_approved": EVENT_KIND_TACTICAL,
    "tactical_recommendation_rejected": EVENT_KIND_TACTICAL,
    "tactical_autonomous_paused": EVENT_KIND_TACTICAL,
    "tactical_autonomous_resumed": EVENT_KIND_TACTICAL,
    "tactical_capture_annex_written": EVENT_KIND_CAPTURE,
    "tactical_capture_annex_empty": EVENT_KIND_CAPTURE,
    "tactical_capture_snapshot": EVENT_KIND_CAPTURE,
    "tactical_switch": EVENT_KIND_CAPTURE,
    "tactical_assignment": EVENT_KIND_CAPTURE,
    "tactical_lock": EVENT_KIND_CAPTURE,
    "tactical_pause_resume": EVENT_KIND_CAPTURE,
}


def classify_event_kind(command_type: str) -> str:
    mapped = _COMMAND_TYPE_TO_KIND.get(command_type)
    if mapped is not None:
        return mapped
    if command_type in ALLOWED_COMMANDS:
        return EVENT_KIND_USER
    if command_type in RUNTIME_SUBCOMMANDS:
        return EVENT_KIND_ADAPTER
    return EVENT_KIND_LIFECYCLE


def is_user_command(command_type: str) -> bool:
    return classify_event_kind(command_type) == EVENT_KIND_USER
