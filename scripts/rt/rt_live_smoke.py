#!/usr/bin/env python3
"""Live RT sandbox integration smoke (maintainer-only, loopback bridge).

Runs: start_sim -> subscribe_telemetry -> spawn_attacker -> poll mirror ->
reset_sim -> spawn_attacker -> stop_sim, then checks for orphan processes.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[2]
_DEFAULT_URL = "http://127.0.0.1:18765/v1/command"
_ORPHAN_PATTERNS = (
    "ros2 launch rt_sandbox_gz",
    "gz sim",
    "rt_sandbox_gz_bridge_node",
)


def send_command(
    command_type: str,
    *,
    url: str,
    session_id: str | None = None,
    payload: dict | None = None,
    timeout_s: float = 120.0,
) -> dict[str, Any]:
    body: dict[str, Any] = {
        "schema": "rt_bridge_request_v1",
        "command_type": command_type,
        "command_id": str(uuid.uuid4()),
        "issued_by": "rt_live_smoke",
        "authority_scope": "rt_sandbox_prototype",
    }
    if session_id:
        body["session_id"] = session_id
    if payload is not None:
        body["payload"] = payload
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def pgrep_orphans() -> list[str]:
    lines: list[str] = []
    for pattern in _ORPHAN_PATTERNS:
        proc = subprocess.run(
            ["pgrep", "-af", pattern],
            capture_output=True,
            text=True,
        )
        for line in proc.stdout.splitlines():
            line = line.strip()
            if line and "rt_live_smoke" not in line:
                lines.append(line)
    return sorted(set(lines))


def pull_telemetry(
    *,
    base_url: str,
    session_id: str,
    subscription_id: str,
    max_events: int = 20,
) -> dict[str, Any]:
    url = (
        f"{base_url.replace('/v1/command', '')}/v1/telemetry/pull"
        f"?session_id={session_id}&subscription_id={subscription_id}&max_events={max_events}"
    )
    with urllib.request.urlopen(url, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def entity_pose_mirror_from_events(
    events: list[dict[str, Any]],
    *,
    entity_id: str | None = None,
) -> dict[str, Any] | None:
    mirrors: list[dict[str, Any]] = []
    for ev in events:
        if ev.get("channel") != "entity_pose_mirror":
            continue
        payload = ev.get("payload")
        if isinstance(payload, dict):
            mirrors.append(payload)
    if not mirrors:
        return None
    if entity_id:
        for payload in reversed(mirrors):
            entities = list(payload.get("entities") or [])
            if any(e.get("entity_id") == entity_id for e in entities):
                return payload
    for payload in reversed(mirrors):
        if list(payload.get("entities") or []):
            return payload
    return mirrors[-1]


def preflight_cleanup_sessions(url: str) -> None:
    try:
        listed = send_command("list_sessions", url=url, timeout_s=10.0)
    except urllib.error.URLError:
        return
    if not listed.get("ok"):
        return
    for row in listed.get("sessions") or []:
        sid = str(row.get("session_id", ""))
        if not sid:
            continue
        state = str(row.get("state", ""))
        if state in {"running", "paused", "created"}:
            send_command("stop_sim", url=url, session_id=sid, timeout_s=60.0)
        send_command("discard_session", url=url, session_id=sid, timeout_s=30.0)


def preflight_kill_stale_orphans() -> None:
    for pattern in _ORPHAN_PATTERNS:
        subprocess.run(["pkill", "-f", pattern], check=False)
    time.sleep(1.0)


def run_smoke(
    *,
    url: str,
    trace_path: Path | None,
    wait_after_stop_s: float,
) -> dict[str, Any]:
    trace: list[dict[str, Any]] = []
    checks: dict[str, bool] = {
        "bridge_reachable": False,
        "start_sim_ok": False,
        "gazebo_launched": False,
        "subscribe_ok": False,
        "spawn_attacker_ok": False,
        "telemetry_has_attacker": False,
        "reset_sim_ok": False,
        "respawn_ok": False,
        "stop_sim_ok": False,
        "no_orphans_after_stop": False,
    }
    blockers: list[str] = []
    session_id: str | None = None
    subscription_id: str | None = None

    def record(step: str, result: dict[str, Any]) -> None:
        trace.append({"step": step, "result": result})

    pre_orphans = pgrep_orphans()
    record("pre_orphans", {"lines": pre_orphans})
    preflight_kill_stale_orphans()
    preflight_cleanup_sessions(url)

    try:
        start = send_command("start_sim", url=url, timeout_s=120.0)
    except urllib.error.URLError as exc:
        blockers.append(f"bridge_unreachable: {exc}")
        record("start_sim", {"ok": False, "error": str(exc)})
        return _finish(checks, blockers, trace, trace_path)

    checks["bridge_reachable"] = True
    record("start_sim", start)
    if not start.get("ok"):
        blockers.append(f"start_sim_failed: {start.get('error_code')} {start.get('message')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["start_sim_ok"] = True
    session_id = str(start["session_id"])

    edit = send_command(
        "set_editing_session",
        url=url,
        payload={"session_id": session_id},
    )
    record("set_editing_session", edit)
    if not edit.get("ok"):
        blockers.append(f"set_editing_session_failed: {edit.get('error_code')}")
        return _finish(checks, blockers, trace, trace_path)

    health = send_command(
        "send_runtime_command",
        url=url,
        session_id=session_id,
        payload={"sub_command": "adapter_health"},
        timeout_s=30.0,
    )
    record("adapter_health", health)
    runtime_health = health.get("runtime_health") or {}
    if health.get("ok") and runtime_health.get("adapter_alive"):
        checks["gazebo_launched"] = True
    elif pgrep_orphans():
        checks["gazebo_launched"] = True
    else:
        blockers.append(f"gazebo_not_live: {runtime_health or health.get('error_code')}")

    sub = send_command(
        "subscribe_telemetry",
        url=url,
        session_id=session_id,
        payload={"channels": ["entity_pose_mirror", "lifecycle_state", "session_health"]},
    )
    record("subscribe_telemetry", sub)
    if not sub.get("ok"):
        blockers.append(f"subscribe_failed: {sub.get('error_code')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["subscribe_ok"] = True
    subscription_id = str(sub.get("subscription_id", ""))

    spawn1 = send_command(
        "spawn_attacker",
        url=url,
        session_id=session_id,
        payload={"pose": {"x": 5.0, "y": -3.0, "z": 20.0, "yaw_deg": 45.0}},
        timeout_s=60.0,
    )
    record("spawn_attacker_1", spawn1)
    if not spawn1.get("ok"):
        blockers.append(f"spawn_attacker_failed: {spawn1.get('error_code')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["spawn_attacker_ok"] = True
    entity_id = str(spawn1.get("entity_id", ""))

    time.sleep(1.0)
    poll = send_command(
        "send_runtime_command",
        url=url,
        session_id=session_id,
        payload={"sub_command": "adapter_poll_telemetry"},
    )
    record("adapter_poll_telemetry", poll)

    pull_base = url.replace("/v1/command", "")
    pull_out: dict[str, Any] = {"events": []}
    if subscription_id:
        try:
            pull_out = pull_telemetry(
                base_url=pull_base,
                session_id=session_id,
                subscription_id=subscription_id,
            )
        except urllib.error.URLError as exc:
            blockers.append(f"telemetry_pull_failed: {exc}")
    record("telemetry_pull", pull_out)

    events = list(pull_out.get("events") or sub.get("initial_events") or [])
    mirror = entity_pose_mirror_from_events(events, entity_id=entity_id)
    if subscription_id and not list((mirror or {}).get("entities") or []):
        for _ in range(5):
            send_command(
                "send_runtime_command",
                url=url,
                session_id=session_id,
                payload={"sub_command": "adapter_poll_telemetry"},
            )
            time.sleep(0.5)
            try:
                pull_out = pull_telemetry(
                    base_url=pull_base,
                    session_id=session_id,
                    subscription_id=subscription_id,
                )
                events.extend(pull_out.get("events") or [])
                mirror = entity_pose_mirror_from_events(events, entity_id=entity_id)
                if mirror and list(mirror.get("entities") or []):
                    break
            except urllib.error.URLError:
                pass
    if not list((mirror or {}).get("entities") or []):
        verify_sub = send_command(
            "subscribe_telemetry",
            url=url,
            session_id=session_id,
            payload={"channels": ["entity_pose_mirror"]},
        )
        record("verify_entity_pose_mirror", verify_sub)
        mirror = entity_pose_mirror_from_events(
            list(verify_sub.get("initial_events") or []),
            entity_id=entity_id,
        )
    if mirror:
        entities = list(mirror.get("entities") or [])
        attacker = next((e for e in entities if e.get("entity_id") == entity_id), None)
        if attacker and (attacker.get("position") or attacker.get("pose")):
            checks["telemetry_has_attacker"] = True
    if not checks["telemetry_has_attacker"]:
        blockers.append("entity_pose_mirror_missing_attacker")

    reset = send_command("reset_sim", url=url, session_id=session_id)
    record("reset_sim", reset)
    if not reset.get("ok") or reset.get("state") != "running":
        blockers.append(f"reset_sim_failed: {reset.get('error_code')} state={reset.get('state')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["reset_sim_ok"] = True

    spawn2 = send_command(
        "spawn_attacker",
        url=url,
        session_id=session_id,
        payload={"pose": {"x": -2.0, "y": 4.0, "z": 15.0, "yaw_deg": 0.0}},
        timeout_s=60.0,
    )
    record("spawn_attacker_2", spawn2)
    if not spawn2.get("ok"):
        blockers.append(f"respawn_failed: {spawn2.get('error_code')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["respawn_ok"] = True

    stop = send_command("stop_sim", url=url, session_id=session_id, timeout_s=60.0)
    record("stop_sim", stop)
    if not stop.get("ok") or stop.get("state") != "stopped":
        blockers.append(f"stop_sim_failed: {stop.get('error_code')} state={stop.get('state')}")
        return _finish(checks, blockers, trace, trace_path)

    checks["stop_sim_ok"] = True

    time.sleep(max(0.5, wait_after_stop_s))
    post_orphans = pgrep_orphans()
    record("post_orphans", {"lines": post_orphans})
    checks["no_orphans_after_stop"] = len(post_orphans) == 0
    if post_orphans:
        blockers.append(f"orphans_after_stop: {post_orphans}")

    return _finish(checks, blockers, trace, trace_path)


def _finish(
    checks: dict[str, bool],
    blockers: list[str],
    trace: list[dict[str, Any]],
    trace_path: Path | None,
) -> dict[str, Any]:
    report = {
        "ok": all(checks.values()) and not blockers,
        "checks": checks,
        "blockers": blockers,
        "trace": trace,
    }
    if trace_path is not None:
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="RT live integration smoke")
    parser.add_argument("--url", default=_DEFAULT_URL)
    parser.add_argument(
        "--trace",
        default=str(_REPO / "fixtures" / "rt_sandbox" / "live_smoke_trace_v1.json"),
        help="Write golden trace JSON (pass --trace '' to skip)",
    )
    parser.add_argument(
        "--wait-after-stop-s",
        type=float,
        default=3.0,
        help="Seconds to wait before orphan pgrep check",
    )
    args = parser.parse_args()
    trace_path = Path(args.trace) if args.trace else None
    report = run_smoke(
        url=args.url,
        trace_path=trace_path,
        wait_after_stop_s=args.wait_after_stop_s,
    )
    print(json.dumps({k: v for k, v in report.items() if k != "trace"}, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
