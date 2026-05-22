#!/usr/bin/env python3
"""Minimal ASCII top-down entity placement view (dev-only, local loopback).

For live telemetry polling use scripts/rt/rt_telemetry_viz.py instead.
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path

# Reuse client transport
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from rt_bridge_client import DEFAULT_URL, send_command  # noqa: E402


def _grid_char(x: float, y: float, entities: list[dict], *, scale: float = 0.05) -> str:
    """Map world coords to a small ASCII grid."""
    gx = int((x + 500) * scale)
    gy = int((y + 500) * scale)
    width, height = 40, 20
    if gx < 0 or gx >= width or gy < 0 or gy >= height:
        return " "
    marks = {"radar": "R", "interceptor": "I", "drone": "D", "waypoint_marker": "W"}
    for ent in entities:
        pose = ent.get("pose") or {}
        ex = int((float(pose.get("x", 0)) + 500) * scale)
        ey = int((float(pose.get("y", 0)) + 500) * scale)
        if ex == gx and ey == gy:
            return marks.get(ent.get("entity_type", ""), "?")
    return "."


def render_world(entities: list[dict], summary: dict | None) -> str:
    lines = ["RT world (top-down, prototype units)", ""]
    if summary:
        lines.append(
            f"entities={summary.get('entity_count', 0)} revision={summary.get('revision', 0)}"
        )
        by_type = summary.get("by_type") or {}
        if by_type:
            lines.append("by_type: " + ", ".join(f"{k}={v}" for k, v in sorted(by_type.items()) if v))
        lines.append("")
    width, height = 40, 20
    scale = width / 1000.0
    for row in range(height):
        row_chars = []
        for col in range(width):
            x = (col / scale) - 500
            y = (row / scale) - 500
            row_chars.append(_grid_char(x, y, entities, scale=scale))
        lines.append("".join(row_chars))
    lines.append("")
    lines.append("Legend: R=radar I=interceptor D=drone W=waypoint . empty")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="RT sandbox ASCII world view")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--session-id", required=True)
    parser.add_argument(
        "--from-response",
        default=None,
        help="JSON file from last bridge response (uses entities/world_summary fields)",
    )
    args = parser.parse_args()

    if args.from_response:
        data = json.loads(Path(args.from_response).read_text(encoding="utf-8"))
        entities = data.get("entities") or []
        summary = data.get("world_summary")
        print(render_world(entities, summary))
        return 0

    # Probe via spawn noop — use last entity command output stored externally.
    # Dev workflow: pipe rt_bridge_client spawn output to --from-response
    print(
        "Provide --from-response with JSON from rt_bridge_client (spawn/move/delete).\n"
        "Example: rt_bridge_client.py spawn_entity ... > /tmp/last.json && "
        "rt_world_viz.py --session-id ID --from-response /tmp/last.json",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
