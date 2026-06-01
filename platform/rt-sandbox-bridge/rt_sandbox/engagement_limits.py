"""Shared engagement limits (bridge path). See simulation/engagement_limits.py."""

from __future__ import annotations

import sys
from pathlib import Path

_SIM = Path(__file__).resolve().parents[3] / 'simulation'
if str(_SIM) not in sys.path:
    sys.path.insert(0, str(_SIM))

from engagement_limits import (  # noqa: E402
    EngagementLimits,
    default_engagement_limits,
    load_engagement_limits,
)

__all__ = [
    'EngagementLimits',
    'default_engagement_limits',
    'load_engagement_limits',
]
