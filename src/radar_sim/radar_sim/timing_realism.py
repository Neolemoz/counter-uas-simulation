"""Optional sensor timing helpers (decimation, transport delay)."""

from __future__ import annotations

import random


def should_publish_on_callback(callback_index: int, publish_every_n: int) -> bool:
    """Return True when this input callback should produce a detection publish.

    ``publish_every_n=2`` publishes on callbacks 2, 4, 6, … (every second callback).
    ``publish_every_n=1`` preserves legacy per-callback behavior.
    """
    n = max(1, int(publish_every_n))
    if n == 1:
        return True
    idx = max(1, int(callback_index))
    return (idx % n) == 0


def transport_delay_s(
    delay_mean_s: float,
    delay_jitter_s: float,
    rng: random.Random | None = None,
) -> float:
    """Deterministic when ``rng`` is seeded; ``jitter=0`` returns ``delay_mean_s`` exactly."""
    mean = max(0.0, float(delay_mean_s))
    jitter = max(0.0, float(delay_jitter_s))
    if jitter <= 0.0:
        return mean
    source = rng if rng is not None else random
    return max(0.0, mean + float(source.uniform(-jitter, jitter)))
