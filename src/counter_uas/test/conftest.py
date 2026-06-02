"""Pytest path setup for platform packages not installed via colcon."""

from __future__ import annotations

import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
_COUNTER_UAS_PKG = _REPO / "src" / "counter_uas"
if _BRIDGE_PKG.is_dir() and str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))
if _COUNTER_UAS_PKG.is_dir() and str(_COUNTER_UAS_PKG) not in sys.path:
    sys.path.insert(0, str(_COUNTER_UAS_PKG))
