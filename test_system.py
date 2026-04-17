#!/usr/bin/env python3
"""
Automated smoke tests: edit config.yaml, rebuild counter_uas, launch stack briefly, grep logs.

Run from workspace root (after colcon build at least once):
  python3 test_system.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parent
CONFIG_PATH = WORKSPACE / "src/counter_uas/config/config.yaml"
INSTALL_SETUP = WORKSPACE / "install/setup.bash"

LAUNCH_SECONDS = 14.0


def config_yaml(
    radar_range: float,
    fusion_threshold: float,
    medium_distance: float,
    high_distance: float,
) -> str:
    return f"""# Parameters for counter_uas bringup (one section per node name).
world_sim_node:
  ros__parameters:
    scenario: single

radar_sim_node:
  ros__parameters:
    scenario: single
    radar:
      range: {radar_range}

camera_sim_node:
  ros__parameters:
    scenario: single

fusion_node:
  ros__parameters:
    scenario: single
    fusion:
      distance_threshold: {fusion_threshold}

tracking_node:
  ros__parameters:
    scenario: single

viz_node:
  ros__parameters:
    scenario: single
    threat:
      medium_distance: {medium_distance}
      high_distance: {high_distance}
"""


def write_config(content: str) -> None:
    CONFIG_PATH.write_text(content, encoding="utf-8")


def _as_str(chunk: str | bytes | None) -> str:
    """TimeoutExpired may attach bytes to stdout/stderr even with text=True."""
    if chunk is None:
        return ""
    if isinstance(chunk, bytes):
        return chunk.decode("utf-8", errors="replace")
    return chunk


def run_colcon_counter_uas() -> None:
    r = subprocess.run(
        ["colcon", "build", "--packages-select", "counter_uas"],
        cwd=WORKSPACE,
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        print("colcon build failed:", r.stderr or r.stdout, file=sys.stderr)
        sys.exit(1)


def run_launch_capture(seconds: float) -> str:
    """Run bringup; kill after `seconds`; return merged stdout+stderr."""
    if not INSTALL_SETUP.is_file():
        print(
            "Missing install/setup.bash — run: colcon build",
            file=sys.stderr,
        )
        sys.exit(1)

    cmd = (
        f"source {INSTALL_SETUP} && "
        "ros2 launch counter_uas bringup.launch.py"
    )
    try:
        r = subprocess.run(
            ["bash", "-lc", cmd],
            cwd=WORKSPACE,
            capture_output=True,
            text=True,
            timeout=seconds,
        )
        return _as_str(r.stdout) + _as_str(r.stderr)
    except subprocess.TimeoutExpired as e:
        return _as_str(e.stdout) + _as_str(e.stderr)


def check(name: str, log: str, needle: str) -> bool:
    ok = needle in log
    print(f"  [{name}] {'PASS' if ok else 'FAIL'}  (expected substring: {needle!r})")
    return ok


def main() -> None:
    if not CONFIG_PATH.is_file():
        print(f"Missing config: {CONFIG_PATH}", file=sys.stderr)
        sys.exit(1)

    backup = CONFIG_PATH.read_text(encoding="utf-8")
    results: list[tuple[str, bool]] = []

    try:
        # --- Test 1: tiny radar range → out of range messages ---
        print("Test 1: small radar range → 'Target out of range'")
        write_config(
            config_yaml(
                radar_range=0.5,
                fusion_threshold=6.0,
                medium_distance=8.0,
                high_distance=5.0,
            )
        )
        run_colcon_counter_uas()
        log1 = run_launch_capture(LAUNCH_SECONDS)
        results.append(
            (
                "radar range",
                check("radar range", log1, "Target out of range"),
            )
        )

        # --- Test 2: tiny fusion threshold → separate streams, "Radar only" in log ---
        print("\nTest 2: small fusion threshold → 'Radar only'")
        write_config(
            config_yaml(
                radar_range=20.0,
                fusion_threshold=0.001,
                medium_distance=8.0,
                high_distance=5.0,
            )
        )
        run_colcon_counter_uas()
        log2 = run_launch_capture(LAUNCH_SECONDS)
        results.append(
            (
                "fusion threshold",
                check("fusion threshold", log2, "Radar only"),
            )
        )

        # --- Test 3: large threat bounds → horizontal distance stays HIGH ---
        print("\nTest 3: large threat thresholds → 'threat: HIGH'")
        write_config(
            config_yaml(
                radar_range=20.0,
                fusion_threshold=6.0,
                medium_distance=500.0,
                high_distance=200.0,
            )
        )
        run_colcon_counter_uas()
        log3 = run_launch_capture(LAUNCH_SECONDS)
        results.append(
            (
                "threat thresholds",
                check("threat thresholds", log3, "threat: HIGH"),
            )
        )

    finally:
        write_config(backup)
        run_colcon_counter_uas()

    print("\n--- Summary ---")
    all_ok = True
    for label, ok in results:
        status = "PASS" if ok else "FAIL"
        print(f"  {label}: {status}")
        all_ok = all_ok and ok
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
