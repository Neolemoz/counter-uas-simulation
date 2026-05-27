"""World revision hint policy exports (PLAT-RT-R3d).

Read-only helpers for docs and tests. See docs/evaluation/rt_world_revision_hint_policy_v1.md.
"""

from __future__ import annotations

_REVISION_COUNTER_ROLES: tuple[dict[str, str], ...] = (
    {
        "counter": "world.revision",
        "owner": "WorldStateStore",
        "authority": "command_authoritative",
    },
    {
        "counter": "world_summary.sync_revision",
        "owner": "pose_sync.summary",
        "authority": "command_authoritative",
    },
    {
        "counter": "EntitySyncEntry.sync_revision",
        "owner": "PoseSyncMirror",
        "authority": "explanatory_sync",
    },
    {
        "counter": "sync_seq",
        "owner": "adapter_worker / PoseSyncMirror",
        "authority": "explanatory_sync",
    },
    {
        "counter": "telemetry_revision",
        "owner": "TelemetryMirror",
        "authority": "explanatory_telemetry",
    },
    {
        "counter": "world_revision_hint",
        "owner": "adapter_worker poll_telemetry",
        "authority": "explanatory_telemetry",
    },
    {
        "counter": "conversion_revision",
        "owner": "capture_normalize",
        "authority": "replay_boundary_scoped",
    },
)

_WORLD_REVISION_HINT_KEYS = frozenset({"telemetry_seq", "sync_seq"})


def revision_counter_roles() -> list[dict[str, str]]:
    """Return read-only counter ownership/authority rows for docs and tests."""
    return [dict(row) for row in _REVISION_COUNTER_ROLES]


def expected_world_revision_hint_keys() -> frozenset[str]:
    """Keys present in adapter world_revision_hint bundles."""
    return _WORLD_REVISION_HINT_KEYS
