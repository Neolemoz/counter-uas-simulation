"""Deny-by-default ROS topic allow-list for RT sandbox adapter (PLAT-RT-G2)."""

from __future__ import annotations

import re

SESSION_PREFIX_RE = re.compile(r"^/rt_sandbox/s_[0-9a-fA-F_]{36}/")

BLOCKED_TOPIC_EXACT = frozenset(
    {
        "/tracks/state",
        "/fused_detections",
    }
)

BLOCKED_TOPIC_PREFIXES = (
    "/tracks/",
    "/corpus_",
    "/federation_",
    "/orchestration",
)

BLOCKED_SUBSTRINGS = (
    "corpus_",
    "federation_",
    "orchestration",
    "replay_sa",
    "publish_topic",
)


def ros_session_id(session_id: str) -> str:
    return "s_" + session_id.replace("-", "_")


def session_topic_prefix(session_id: str) -> str:
    return f"/rt_sandbox/{ros_session_id(session_id)}/"


def allowed_session_topics(session_id: str) -> frozenset[str]:
    prefix = session_topic_prefix(session_id)
    return frozenset(
        {
            f"{prefix}entity_pose_cmd",
            f"{prefix}entity_state",
            f"{prefix}clock",
        }
    )


def classify_topic(session_id: str, topic: str) -> str | None:
    """Return error_code if topic forbidden, else None."""
    if not topic or not topic.startswith("/"):
        return "COMMAND_FORBIDDEN"
    low = topic.lower()
    for sub in BLOCKED_SUBSTRINGS:
        if sub in low:
            return "COMMAND_FORBIDDEN"
    if topic in BLOCKED_TOPIC_EXACT:
        return "COMMAND_FORBIDDEN"
    for prefix in BLOCKED_TOPIC_PREFIXES:
        if topic.startswith(prefix):
            return "COMMAND_FORBIDDEN"
    expected_prefix = session_topic_prefix(session_id)
    if not topic.startswith(expected_prefix):
        return "COMMAND_FORBIDDEN"
    if topic not in allowed_session_topics(session_id):
        return "COMMAND_FORBIDDEN"
    return None
