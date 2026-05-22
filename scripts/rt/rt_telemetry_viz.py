#!/usr/bin/env python3
"""Live RT telemetry ASCII view via loopback pull (dev-only — not SA viewer)."""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

DEFAULT_BASE = "http://127.0.0.1:18765"


def pull_telemetry(
    *,
    base_url: str,
    session_id: str,
    subscription_id: str,
    max_events: int = 10,
) -> dict:
    qs = urllib.parse.urlencode(
        {
            "session_id": session_id,
            "subscription_id": subscription_id,
            "max_events": max_events,
        }
    )
    url = f"{base_url.rstrip('/')}/v1/telemetry/pull?{qs}"
    with urllib.request.urlopen(url, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _entities_from_events(events: list[dict]) -> list[dict]:
    for ev in reversed(events):
        if ev.get("channel") == "entity_pose_mirror":
            payload = ev.get("payload") or {}
            return payload.get("entities") or []
    return []


def _summary_from_events(events: list[dict]) -> dict | None:
    for ev in reversed(events):
        if ev.get("channel") == "world_summary":
            return ev.get("payload")
        if ev.get("channel") == "session_health":
            continue
    return None


def _state_from_events(events: list[dict]) -> str:
    for ev in reversed(events):
        if ev.get("channel") in ("session_health", "lifecycle_state"):
            payload = ev.get("payload") or {}
            if "state" in payload:
                return str(payload["state"])
    return "unknown"


def render_grid(entities: list[dict], summary: dict | None, state: str) -> str:
    lines = [
        "RT telemetry view (read-only, prototype)",
        f"session_state={state}",
        "",
    ]
    if summary:
        lines.append(
            f"entities={summary.get('entity_count', 0)} revision={summary.get('revision', 0)}"
        )
        lines.append("")
    width, height = 40, 20
    scale = width / 1000.0
    marks = {"radar": "R", "interceptor": "I", "drone": "D", "waypoint_marker": "W"}
    for row in range(height):
        row_chars = []
        for col in range(width):
            ch = "."
            for ent in entities:
                pose = ent.get("pose") or {}
                ex = int((float(pose.get("x", 0)) + 500) * scale)
                ey = int((float(pose.get("y", 0)) + 500) * scale)
                if ex == col and ey == row:
                    ch = marks.get(ent.get("entity_type", ""), "?")
            row_chars.append(ch)
        lines.append("".join(row_chars))
    lines.append("")
    lines.append("Legend: R=radar I=interceptor D=drone W=waypoint")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="RT telemetry live ASCII view")
    parser.add_argument("--base-url", default=DEFAULT_BASE)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--subscription-id", required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--once", action="store_true", help="Single pull then exit")
    args = parser.parse_args()

    try:
        while True:
            out = pull_telemetry(
                base_url=args.base_url,
                session_id=args.session_id,
                subscription_id=args.subscription_id,
            )
            if not out.get("ok"):
                print(json.dumps(out, indent=2), file=sys.stderr)
                return 1
            events = out.get("events") or []
            entities = _entities_from_events(events)
            summary = _summary_from_events(events)
            state = _state_from_events(events)
            print(render_grid(entities, summary, state))
            print(f"--- drained {out.get('drained_count', 0)} events ---")
            if args.once:
                break
            time.sleep(args.interval)
    except urllib.error.URLError as exc:
        print(json.dumps({"ok": False, "error_code": "BRIDGE_DISCONNECTED", "message": str(exc)}))
        return 1
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
