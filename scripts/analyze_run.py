#!/usr/bin/env python3
"""
analyze_run.py — parse a Gazebo simulation log and summarize 3 key metrics:

  1. Miss distance trend  (decreasing → interceptor is closing in)
  2. Time-to-intercept    (stability → low σ means predict solution is steady)
  3. Trajectory smoothness (heading change °/step → low = smooth)

Usage
-----
  # Analyze an existing log:
  python3 scripts/analyze_run.py runs/logs/<run>.log

  # Run simulation then analyze immediately:
  python3 scripts/analyze_run.py --run [--scenario single|multi] [--timeout-s 20]

  # Noisy run then analyze:
  python3 scripts/analyze_run.py --run --launch-args \
      "use_noisy_measurement:=true noise_std_m:=0.5 dropout_prob:=0.05 noise_seed:=1"
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
INSTALL_SETUP = WORKSPACE / "install" / "setup.bash"
RUNS_DIR = WORKSPACE / "runs" / "logs"

# ── regex patterns (match anywhere in the line; ros2 launch prefixes stripped) ─

# [METRICS] id-row:  "id=interceptor_0  | dist=5.123 m | t_go=1.234 s | vel=4.5 m/s | mode=predict | min_miss=0.34 m"
_METRICS_ROW_RE = re.compile(
    r"id=(?P<iid>\S+)"
    r".*?dist=(?P<dist>[0-9]+(?:\.[0-9]+)?)\s*m"
    r".*?t_go=(?P<tgo>[0-9]+(?:\.[0-9]+)?|n/a)\s*"
    r".*?vel=(?P<vel>[0-9]+(?:\.[0-9]+)?)\s*m/s"
    r".*?mode=(?P<mode>\S+)"
)

# [Intercept Debug] sub-lines
_DEBUG_DIST_RE  = re.compile(r"^distance=(?P<v>[0-9]+(?:\.[0-9]+)?)$")
_DEBUG_THIT_RE  = re.compile(r"^t_hit=(?P<v>[0-9]+(?:\.[0-9]+)?)$")
_DEBUG_VEL_RE   = re.compile(r"^cmd_vel=\((?P<vx>-?[0-9.]+),(?P<vy>-?[0-9.]+),(?P<vz>-?[0-9.]+)\)$")
_DEBUG_MODE_RE  = re.compile(r"^mode=(?P<v>\S+)$")
_DEBUG_BLOCK_RE = re.compile(r"\[Intercept Debug\]")

# hit / miss (existing tags, unchanged)
_HIT_RE     = re.compile(r"\[HIT\]")
_MINMISS_RE = re.compile(r"\[min_miss\]\s*=\s*(?P<v>[0-9]+(?:\.[0-9]+)?)\s*m\b")
_HITHR_RE   = re.compile(r"hit_threshold\s*=\s*(?P<v>[0-9]+(?:\.[0-9]+)?)\s*m")
_MINMISS_IN_HIT_RE = re.compile(r"\bmin_miss=(?P<v>[0-9]+(?:\.[0-9]+)?)\s*m\b")

# optional evaluation-layer tags (used by summarize_run.py; keep parsing lightweight here too)
_LAYER_RE = re.compile(r"\[LAYER\]\s+(?P<layer>\S+)\b")
_LAYER_IN_HIT_RE = re.compile(r"\blayer=(?P<layer>\S+)\b")

# optional feasibility / margin tags (best-effort; may not exist in all logs)
_MARGIN_RE = re.compile(r"\bmargin\s*=\s*(?P<v>-?[0-9]+(?:\.[0-9]+)?)\s*s\b")
_TTI_TARGET_RE = re.compile(r"\bTTI_target\s*[:=]\s*(?P<v>-?[0-9]+(?:\.[0-9]+)?)\b")
_T_REQUIRED_RE = re.compile(r"\bT_required\s*[:=]\s*(?P<v>-?[0-9]+(?:\.[0-9]+)?)\b")


def _strip_ros_prefix(line: str) -> str:
    """Remove '[node-N] ' prefix that ros2 launch adds to every output line."""
    s = line.strip()
    m = re.match(r"^\[[^\]]+\]\s+", s)
    return s[m.end():] if m else s


def _norm3(vx: float, vy: float, vz: float) -> float:
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def _angle_deg(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    na, nb = _norm3(*a), _norm3(*b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    cos_t = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_t))))


# ── parser ───────────────────────────────────────────────────────────────────

def parse_log(text: str) -> dict:
    """Return a dict with all time-series data extracted from the log."""

    dist_series:   list[float] = []   # from [METRICS] rows
    tgo_series:    list[float] = []   # t_go values (n/a excluded)
    vel_series:    list[float] = []   # commanded velocity magnitude
    cmd_vels:      list[tuple[float, float, float]] = []  # for heading-change calc
    thit_series:   list[float] = []   # t_hit from [Intercept Debug]
    mode_series:   list[str]   = []   # mode from [Intercept Debug]

    hit            = False
    min_miss_m:    float | None = None
    hit_threshold: float | None = None

    in_debug_block = False

    lines = text.splitlines()
    for raw_line in lines:
        s = _strip_ros_prefix(raw_line)

        # ── [METRICS] id-row ─────────────────────────────────────────────────
        m = _METRICS_ROW_RE.search(s)
        if m and "dist=" in s and "t_go=" in s:
            dist_series.append(float(m.group("dist")))
            tgo_raw = m.group("tgo")
            if tgo_raw != "n/a":
                tgo_series.append(float(tgo_raw))
            vel_series.append(float(m.group("vel")))
            in_debug_block = False
            continue

        # ── [Intercept Debug] block detection ────────────────────────────────
        if _DEBUG_BLOCK_RE.search(s):
            in_debug_block = True
            continue

        if in_debug_block:
            dm = _DEBUG_DIST_RE.match(s)
            if dm:
                continue   # distance already captured from [METRICS]

            tm = _DEBUG_THIT_RE.match(s)
            if tm:
                thit_series.append(float(tm.group("v")))
                continue

            vm = _DEBUG_VEL_RE.match(s)
            if vm:
                vx, vy, vz = float(vm.group("vx")), float(vm.group("vy")), float(vm.group("vz"))
                cmd_vels.append((vx, vy, vz))
                continue

            mm = _DEBUG_MODE_RE.match(s)
            if mm:
                mode_series.append(mm.group("v"))
                continue

            # blank or unrecognized line → end of debug block
            if not s or s.startswith("["):
                in_debug_block = False

        # ── existing tags ─────────────────────────────────────────────────────
        if _HIT_RE.search(s):
            hit = True
            hm = _MINMISS_IN_HIT_RE.search(s)
            if hm:
                v = float(hm.group("v"))
                if min_miss_m is None or v < min_miss_m:
                    min_miss_m = v

        m2 = _MINMISS_RE.search(s)
        if m2:
            v = float(m2.group("v"))
            if min_miss_m is None or v < min_miss_m:
                min_miss_m = v

        m3 = _HITHR_RE.search(s)
        if m3:
            hit_threshold = float(m3.group("v"))

    # heading-change series (°/step) from consecutive cmd_vel vectors
    heading_changes: list[float] = []
    for i in range(1, len(cmd_vels)):
        heading_changes.append(_angle_deg(cmd_vels[i - 1], cmd_vels[i]))

    return {
        "hit":              hit,
        "hit_threshold_m":  hit_threshold,
        "min_miss_m":       min_miss_m,
        "dist_series":      dist_series,
        "tgo_series":       tgo_series,
        "vel_series":       vel_series,
        "thit_series":      thit_series,
        "mode_series":      mode_series,
        "heading_changes":  heading_changes,
    }


def parse_run_to_result(log_path: str) -> dict:
    """
    Parse a Gazebo run log into a compact, structured result dict.

    This is intended to mirror the "offline evaluation" shape (success/miss/time),
    while reusing the existing parsing logic in this file.
    """
    path = Path(str(log_path))
    text = path.read_text(encoding="utf-8", errors="replace")
    data = parse_log(text)

    # ── required fields ─────────────────────────────────────────────────────
    success = bool(data.get("hit", False))

    mm = data.get("min_miss_m", None)
    if mm is None:
        dist_series = data.get("dist_series", []) or []
        mm_val = float(min(dist_series)) if dist_series else float("nan")
        mm_note = "miss_distance from dist_series min" if dist_series else "miss_distance unavailable"
    else:
        mm_val = float(mm)
        mm_note = "miss_distance from [min_miss]"

    tgo = data.get("tgo_series", []) or []
    thit = data.get("thit_series", []) or []
    intercept_time_s: float | None
    if tgo:
        intercept_time_s = float(tgo[-1])
        t_note = "intercept_time from last t_go sample"
    elif thit:
        intercept_time_s = float(thit[-1])
        t_note = "intercept_time from last t_hit sample"
    else:
        intercept_time_s = None
        t_note = "intercept_time unavailable"

    # ── optional layer_at_hit + time_margin ─────────────────────────────────
    layer_last: str | None = None
    layer_at_hit: str | None = None

    time_margin_s: float | None = None
    tti_target_s: float | None = None
    t_required_s: float | None = None

    for raw_line in text.splitlines():
        s = _strip_ros_prefix(raw_line)

        lm = _LAYER_RE.search(s)
        if lm:
            layer_last = lm.group("layer")

        if _HIT_RE.search(s) and layer_at_hit is None:
            # prefer explicit "layer=..." in the [HIT] line; else fall back to last [LAYER]
            lhit = _LAYER_IN_HIT_RE.search(s)
            if lhit:
                layer_at_hit = lhit.group("layer")
            else:
                layer_at_hit = layer_last

        if time_margin_s is None:
            mmg = _MARGIN_RE.search(s)
            # Avoid misinterpreting controller tuning strings like "margin=0.3s window=2.0s".
            # Only treat "margin=" as a time margin if the same line also carries feasibility context.
            if mmg and ("TTI_target" in s or "T_required" in s):
                time_margin_s = float(mmg.group("v"))

        if tti_target_s is None:
            m_tti = _TTI_TARGET_RE.search(s)
            if m_tti:
                tti_target_s = float(m_tti.group("v"))

        if t_required_s is None:
            m_tr = _T_REQUIRED_RE.search(s)
            if m_tr:
                t_required_s = float(m_tr.group("v"))

        if time_margin_s is None and (tti_target_s is not None) and (t_required_s is not None):
            time_margin_s = float(tti_target_s - t_required_s)

    if layer_at_hit is None:
        layer_note = "layer_at_hit unavailable"
    else:
        layer_note = "layer_at_hit parsed"

    margin_note = "time_margin unavailable" if time_margin_s is None else "time_margin parsed"

    notes = "; ".join([mm_note, t_note, layer_note, margin_note])

    return {
        "success": success,
        "miss_distance_m": mm_val,
        "intercept_time_s": intercept_time_s,
        "time_margin_s": time_margin_s,
        "layer_at_hit": layer_at_hit,
        "notes": notes,
    }


# ── statistics helpers ────────────────────────────────────────────────────────

def _mean(xs: list[float]) -> float:
    return sum(xs) / len(xs) if xs else float("nan")

def _std(xs: list[float]) -> float:
    if len(xs) < 2:
        return float("nan")
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))

def _pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    idx = (len(s) - 1) * p / 100
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)

def _segments(xs: list[float], n: int = 4) -> list[tuple[int, int, float, float]]:
    """Split xs into n segments; return (start, end, mean, std)."""
    if not xs:
        return []
    seg = max(1, len(xs) // n)
    result = []
    for i in range(0, len(xs), seg):
        sl = xs[i:i + seg]
        result.append((i, i + len(sl) - 1, _mean(sl), _std(sl)))
    return result


# ── pretty-print ──────────────────────────────────────────────────────────────

def _fmt(v: float, dec: int = 3) -> str:
    return f"{v:.{dec}f}" if math.isfinite(v) else "n/a"

def _bar(v: float, vmax: float, width: int = 20) -> str:
    if not math.isfinite(v) or vmax <= 0:
        return " " * width
    n = max(0, min(width, round(v / vmax * width)))
    return "█" * n + "░" * (width - n)


def print_report(data: dict, run_id: str = "") -> None:
    sep = "=" * 66
    thin = "-" * 66

    print(sep)
    print(f"  INTERCEPTION METRICS REPORT  {run_id}")
    print(sep)

    # ── 0. Hit result ─────────────────────────────────────────────────────────
    hit_s      = "✅ HIT" if data["hit"] else "❌ MISS"
    mm         = data["min_miss_m"]
    thr        = data["hit_threshold_m"]
    mm_s       = f"{mm:.4f} m" if mm is not None else "n/a"
    thr_s      = f"{thr:.4f} m" if thr is not None else "n/a"
    print(f"\n  Result        : {hit_s}")
    print(f"  Min miss dist : {mm_s}  (threshold = {thr_s})")

    dist  = data["dist_series"]
    tgo   = data["tgo_series"]
    hdg   = data["heading_changes"]
    thit  = data["thit_series"]

    # ── 1. Miss distance trend ────────────────────────────────────────────────
    print(f"\n{thin}")
    print("  1. MISS DISTANCE TREND  (should decrease monotonically)")
    print(thin)
    if not dist:
        print("  (no [METRICS] data found — enable intercept_debug:=true)")
    else:
        segs = _segments(dist, 5)
        dmax = max(dist)
        print(f"  Samples : {len(dist)}   first={_fmt(dist[0])} m   last={_fmt(dist[-1])} m"
              f"   min={_fmt(min(dist))} m")
        print()
        for s, e, mu, _ in segs:
            bar = _bar(mu, dmax)
            print(f"  [{s:4d}-{e:4d}]  avg={_fmt(mu):9s} m  |{bar}|")

        # Monotone check: only count steps BEFORE post-HIT freeze.
        # After HIT the simulation stops → all readings identical → exclude from check.
        hit_idx = len(dist)
        if data["hit"]:
            # Find first index where value stays constant until end (post-HIT plateau)
            for i in range(len(dist) - 1, 0, -1):
                if dist[i] != dist[i - 1]:
                    hit_idx = i + 1
                    break
        pre_hit = dist[:hit_idx]
        if len(pre_hit) > 1:
            decreasing = sum(1 for a, b in zip(pre_hit, pre_hit[1:]) if b < a)
            total_pairs = len(pre_hit) - 1
            mono_pct = 100 * decreasing / total_pairs if total_pairs > 0 else 0
            post_note = f'  (post-HIT plateau: {len(dist)-hit_idx} samples excluded)' if data["hit"] and hit_idx < len(dist) else ''
            verdict = "✅ YES" if mono_pct >= 70 else "⚠️  PARTIAL" if mono_pct >= 50 else "❌ NO"
            print(f"\n  Decreasing? {verdict}  ({decreasing}/{total_pairs} steps = {mono_pct:.0f}%){post_note}")
        else:
            print(f"\n  Decreasing? (insufficient pre-HIT samples)")

    # ── 2. Time-to-intercept stability ────────────────────────────────────────
    print(f"\n{thin}")
    print("  2. TIME-TO-INTERCEPT  (predict mode t_go, lower σ = more stable)")
    print(thin)
    if not tgo:
        print("  (no predict-mode solutions found — target may be in pursuit mode only)")
    else:
        segs = _segments(tgo, 4)
        print(f"  Samples : {len(tgo)}   mean={_fmt(_mean(tgo))} s   "
              f"σ={_fmt(_std(tgo))} s   p90={_fmt(_pct(tgo, 90))} s")
        print()
        tgo_max = max(tgo)
        for s, e, mu, sd in segs:
            bar = _bar(mu, tgo_max)
            verdict_seg = "stable" if sd < 0.3 else "jitter" if sd < 0.7 else "unstable"
            print(f"  [{s:4d}-{e:4d}]  mean={_fmt(mu):8s} s  σ={_fmt(sd):6s} s"
                  f"  [{verdict_seg:8s}]  |{bar}|")

        overall_sd = _std(tgo)
        verdict_t = ("✅ STABLE" if overall_sd < 0.4
                     else "⚠️  SOME JITTER" if overall_sd < 0.8
                     else "❌ UNSTABLE")
        print(f"\n  Overall stability: {verdict_t}  (σ = {_fmt(overall_sd)} s)")

    # also show t_hit from [Intercept Debug] if available
    if thit and not tgo:
        print(f"\n  (t_hit from [Intercept Debug]: {len(thit)} samples,"
              f" mean={_fmt(_mean(thit))} s, σ={_fmt(_std(thit))} s)")

    # ── 3. Trajectory smoothness ──────────────────────────────────────────────
    print(f"\n{thin}")
    print("  3. TRAJECTORY SMOOTHNESS  (heading change °/step, lower = smoother)")
    print(thin)
    if not hdg:
        print("  (no cmd_vel series found in [Intercept Debug] blocks)")
    else:
        segs = _segments(hdg, 4)
        hdg_max = max(hdg) if hdg else 1.0
        print(f"  Samples : {len(hdg)}   avg={_fmt(_mean(hdg), 2)}°/step   "
              f"max={_fmt(max(hdg), 2)}°/step   p90={_fmt(_pct(hdg, 90), 2)}°/step")
        print()
        for s, e, mu, _ in segs:
            bar = _bar(mu, hdg_max)
            verdict_seg = "smooth" if mu < 1.0 else "moderate" if mu < 2.5 else "jerky"
            print(f"  [{s:4d}-{e:4d}]  avg={_fmt(mu, 2):6s}°/step  [{verdict_seg:8s}]  |{bar}|")

        avg_hdg = _mean(hdg)
        verdict_h = ("✅ SMOOTH" if avg_hdg < 1.0
                     else "⚠️  MODERATE" if avg_hdg < 2.5
                     else "❌ JERKY")
        print(f"\n  Overall smoothness: {verdict_h}  (avg = {_fmt(avg_hdg, 2)}°/step)")

    # ── 4. Mode distribution ─────────────────────────────────────────────────
    modes = data["mode_series"]
    if modes:
        n_pred = sum(1 for m in modes if "predict" in m)
        n_pur  = len(modes) - n_pred
        pct_p  = 100 * n_pred / len(modes)
        print(f"\n{thin}")
        print(f"  4. GUIDANCE MODE  predict={n_pred} ({pct_p:.0f}%)  pursuit={n_pur} ({100-pct_p:.0f}%)")

    print(f"\n{sep}\n")


# ── run + capture ─────────────────────────────────────────────────────────────

def _run_and_capture(scenario: str, timeout_s: float, launch_args: str | None) -> str:
    """Call run_capture.py, wait for completion, return log path."""
    cmd = [sys.executable, str(WORKSPACE / "scripts" / "run_capture.py"),
           "--scenario", scenario, "--timeout-s", str(timeout_s)]
    if launch_args:
        cmd += ["--launch-args", launch_args]

    print(f"[analyze_run] Starting simulation ({scenario}, {timeout_s}s) …", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode not in (0, 124):
        print(r.stderr, file=sys.stderr)
        raise RuntimeError(f"run_capture.py failed (rc={r.returncode})")

    # run_capture.py prints log path as first line
    lines = r.stdout.strip().splitlines()
    if not lines:
        raise RuntimeError("run_capture.py returned no output")
    log_path = Path(lines[0].strip())
    print(f"[analyze_run] Log saved to: {log_path}", flush=True)
    return str(log_path)


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(
        description="Parse a simulation log and report miss-distance, TTI, and trajectory smoothness.",
    )
    p.add_argument("log", nargs="?", help="Path to .log file (omit if using --run).")
    p.add_argument("--run", action="store_true", help="Run the simulation first then analyze the output.")
    p.add_argument("--scenario", choices=["single", "multi"], default="single")
    p.add_argument("--timeout-s", type=float, default=20.0)
    p.add_argument("--launch-args", type=str, default=None)
    p.add_argument("--json", dest="output_json", action="store_true", help="Also dump raw data as JSON.")
    args = p.parse_args()

    if args.run:
        log_path = _run_and_capture(args.scenario, args.timeout_s, args.launch_args)
    elif args.log:
        log_path = args.log
    else:
        p.print_help()
        return 1

    path = Path(log_path)
    if not path.is_file():
        print(f"Error: file not found: {log_path}", file=sys.stderr)
        return 1

    text = path.read_text(encoding="utf-8", errors="replace")
    data = parse_log(text)

    print_report(data, run_id=path.stem)

    if args.output_json:
        # Emit summary (series are too large; emit statistics only)
        summary = {
            "run_id":           path.stem,
            "hit":              data["hit"],
            "hit_threshold_m":  data["hit_threshold_m"],
            "min_miss_m":       data["min_miss_m"],
            "miss_dist": {
                "n":      len(data["dist_series"]),
                "first":  data["dist_series"][0]  if data["dist_series"] else None,
                "last":   data["dist_series"][-1] if data["dist_series"] else None,
                "min":    min(data["dist_series"]) if data["dist_series"] else None,
            },
            "tgo": {
                "n":    len(data["tgo_series"]),
                "mean": _mean(data["tgo_series"]),
                "std":  _std(data["tgo_series"]),
                "p90":  _pct(data["tgo_series"], 90),
            },
            "trajectory": {
                "n":       len(data["heading_changes"]),
                "avg_deg": _mean(data["heading_changes"]),
                "max_deg": max(data["heading_changes"]) if data["heading_changes"] else None,
                "p90_deg": _pct(data["heading_changes"], 90),
            },
        }
        out_path = path.with_suffix(".analysis.json")
        out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"[analyze_run] JSON saved to: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
