#!/usr/bin/env python3
"""
Run a scenario with a fixed timeout and capture stdout/stderr to runs/logs/.

Phase 1 deliverable: reproducible, machine-parsable log capture (no guidance changes).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
RUNS_DIR = WORKSPACE / "runs" / "logs"
INSTALL_SETUP = WORKSPACE / "install" / "setup.bash"


@dataclass(frozen=True)
class RunMeta:
    run_id: str
    created_utc: str
    workspace: str
    cmd: list[str]
    timeout_s: float
    git_commit: str | None
    git_dirty: bool | None
    notes: str | None


def _utc_ts() -> str:
    return time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime())


def _run_git(args: list[str]) -> str | None:
    try:
        r = subprocess.run(
            ["git", *args],
            cwd=WORKSPACE,
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    if r.returncode != 0:
        return None
    return (r.stdout or "").strip()


def _git_commit() -> str | None:
    return _run_git(["rev-parse", "HEAD"])


def _git_dirty() -> bool | None:
    out = _run_git(["status", "--porcelain"])
    if out is None:
        return None
    return out != ""


def _bash_cmd(source_setup: Path, inner_cmd: str) -> list[str]:
    # Use bash -lc so `source` works and ROS env is loaded.
    return ["bash", "-lc", f"source {source_setup} && {inner_cmd}"]


def run_capture(
    *,
    scenario: str,
    timeout_s: float,
    notes: str | None,
    launch_args: str | None,
) -> tuple[Path, Path, RunMeta, int]:
    if not INSTALL_SETUP.is_file():
        raise FileNotFoundError("Missing install/setup.bash (run colcon build first).")

    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    ts = _utc_ts()
    run_id = f"{ts}_{scenario}"
    log_path = RUNS_DIR / f"{run_id}.log"
    meta_path = RUNS_DIR / f"{run_id}.meta.json"

    if scenario == "single":
        ros_cmd = "ros2 launch gazebo_target_sim gazebo_target.launch.py"
    elif scenario == "multi":
        ros_cmd = "ros2 launch gazebo_target_sim gazebo_target_multi.launch.py"
    else:
        raise ValueError(f"Unknown scenario: {scenario!r}")

    if launch_args:
        ros_cmd = f"{ros_cmd} {launch_args}"

    # Kill any leftover gz/ros2 processes from a previous run so the next
    # launch gets a clean Gazebo instance (otherwise sphere_target won't
    # re-appear because gz remove from the previous HIT is still in effect).
    subprocess.run(
        [
            "bash",
            "-lc",
            # Tear down stale DDS + orphaned nodes so /target/stop and gz state are clean.
            "ros2 daemon stop 2>/dev/null || true; "
            "pkill -9 -f 'gz sim' 2>/dev/null || true; "
            "pkill -9 -f 'ros2 launch' 2>/dev/null || true; "
            "pkill -9 -f 'gazebo_target_sim' 2>/dev/null || true; "
            "pkill -9 -f 'interception_logic_node' 2>/dev/null || true; "
            "pkill -9 -f 'target_controller_node' 2>/dev/null || true; "
            "sleep 1",
        ],
        check=False,
        capture_output=True,
    )

    # Use coreutils `timeout` to ensure the whole launch tree is terminated.
    # (subprocess timeout may leave the launch process tree running.)
    timeout_cmd = f"timeout --signal=TERM {timeout_s:g}s {ros_cmd}"

    cmd = _bash_cmd(INSTALL_SETUP, timeout_cmd)
    # Ensure ROS2 launch logging goes to a writable directory (some environments disallow ~/.ros/log).
    ros_log_dir = WORKSPACE / "runs" / "ros2_logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    home_dir = WORKSPACE / "runs" / "home"
    home_dir.mkdir(parents=True, exist_ok=True)
    meta = RunMeta(
        run_id=run_id,
        created_utc=ts,
        workspace=str(WORKSPACE),
        cmd=cmd,
        timeout_s=float(timeout_s),
        git_commit=_git_commit(),
        git_dirty=_git_dirty(),
        notes=notes,
    )
    meta_path.write_text(json.dumps(asdict(meta), indent=2, sort_keys=True) + "\n", encoding="utf-8")

    with log_path.open("w", encoding="utf-8") as f:
        f.write(f"=== run_id: {run_id} ===\n")
        f.write(f"=== created_utc: {ts} ===\n")
        if meta.git_commit:
            f.write(f"=== git_commit: {meta.git_commit} ===\n")
        if meta.git_dirty is not None:
            f.write(f"=== git_dirty: {meta.git_dirty} ===\n")
        f.write(f"=== scenario: {scenario} ===\n")
        f.write(f"=== timeout_s: {timeout_s} ===\n")
        f.write(f"=== cmd: {cmd} ===\n\n")
        f.flush()

        try:
            env = os.environ.copy()
            env["ROS_LOG_DIR"] = str(ros_log_dir)
            env["HOME"] = str(home_dir)
            r = subprocess.run(
                cmd,
                cwd=WORKSPACE,
                stdout=f,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s + 8.0,
                check=False,
                env=env,
            )
            return log_path, meta_path, meta, int(r.returncode)
        except subprocess.TimeoutExpired:
            f.write("\n=== TIMEOUT ===\n")
            f.flush()
            return log_path, meta_path, meta, 124


def main() -> int:
    p = argparse.ArgumentParser(description="Run a scenario and capture machine-parsable logs.")
    p.add_argument("--scenario", choices=["single", "multi"], default="single")
    p.add_argument("--timeout-s", type=float, default=14.0)
    p.add_argument("--notes", type=str, default=None)
    p.add_argument(
        "--launch-args",
        type=str,
        default=None,
        help='Extra `ros2 launch` arguments, e.g. \'use_noisy_measurement:=true noise_std_m:=0.5\'',
    )
    args = p.parse_args()

    try:
        log_path, meta_path, meta, rc = run_capture(
            scenario=args.scenario,
            timeout_s=args.timeout_s,
            notes=args.notes,
            launch_args=args.launch_args,
        )
    except Exception as e:
        print(f"run_capture failed: {e}", file=sys.stderr)
        return 2

    print(str(log_path))
    print(str(meta_path))
    # Always return 0 for timeouts (common in short smoke runs); parser will decide success later.
    return 0 if rc in (0, 124) else rc


if __name__ == "__main__":
    raise SystemExit(main())

