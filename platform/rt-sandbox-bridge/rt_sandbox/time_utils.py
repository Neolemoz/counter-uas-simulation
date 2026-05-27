"""Shared UTC parsing and poll-age stale helpers (PLAT-RT-R1b)."""

from __future__ import annotations

from datetime import datetime, timezone


def parse_utc(ts: str | None) -> datetime | None:
    if not ts:
        return None
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00"))
    except ValueError:
        return None


def poll_age_seconds(ts: str | None) -> float | None:
    parsed = parse_utc(ts)
    if parsed is None:
        return None
    return (datetime.now(timezone.utc) - parsed).total_seconds()


def is_poll_stale(ts: str | None, threshold_s: float) -> bool:
    if not ts:
        return True
    age = poll_age_seconds(ts)
    if age is None:
        return True
    return age > threshold_s
