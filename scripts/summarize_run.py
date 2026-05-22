#!/usr/bin/env python3
"""
Parse a single run log and emit a small JSON/CSV summary row.

Parses tags from interception_logic_node stdout:
- [HIT]
- [min_miss]
- hit_threshold =
- [LAYER]
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


# ros2 launch prefixes many lines like: "[node-1] [TAG] ...", so parse tags anywhere in the line.
HIT_RE = re.compile(r"\[HIT\]\s+(?P<rest>.*)$")
HIT_THR_RE = re.compile(r"hit_threshold\s*=\s*(?P<thr>[0-9]+(?:\.[0-9]+)?)\s*m")
MIN_MISS_RE = re.compile(r"\[min_miss\]\s*=\s*(?P<mm>[0-9]+(?:\.[0-9]+)?)\s*m\b")
LAYER_RE = re.compile(r"\[LAYER\]\s+(?P<layer>\S+)\b")

LAYER_IN_HIT_RE = re.compile(r"\blayer=(?P<layer>\S+)\b")
MIN_MISS_IN_HIT_RE = re.compile(r"\bmin_miss=(?P<mm>[0-9]+(?:\.[0-9]+)?)\s*m\b")


@dataclass
class RunSummary:
    run_id: str
    hit: bool
    hit_threshold_m: float | None
    min_miss_m: float | None
    layer_at_hit: str
    layer_last: str


def parse_log(text: str, *, run_id: str) -> RunSummary:
    hit = False
    hit_threshold_m: float | None = None
    min_miss_m: float | None = None
    layer_at_hit = "unknown"
    layer_last = "unknown"

    for line in text.splitlines():
        s = line.strip()

        m = MIN_MISS_RE.search(s)
        if m:
            v = float(m.group("mm"))
            if min_miss_m is None or v < min_miss_m:
                min_miss_m = v
            continue

        m = LAYER_RE.search(s)
        if m:
            layer_last = m.group("layer")
            continue

        # hit_threshold lines are not tagged consistently; parse anywhere.
        m = HIT_THR_RE.search(s)
        if m:
            hit_threshold_m = float(m.group("thr"))

        m = HIT_RE.search(s)
        if m:
            hit = True
            rest = m.group("rest")
            lm = LAYER_IN_HIT_RE.search(rest)
            if lm:
                layer_at_hit = lm.group("layer")
            mm = MIN_MISS_IN_HIT_RE.search(rest)
            if mm:
                v = float(mm.group("mm"))
                if min_miss_m is None or v < min_miss_m:
                    min_miss_m = v

    return RunSummary(
        run_id=run_id,
        hit=hit,
        hit_threshold_m=hit_threshold_m,
        min_miss_m=min_miss_m,
        layer_at_hit=layer_at_hit,
        layer_last=layer_last,
    )


def main() -> int:
    p = argparse.ArgumentParser(description="Summarize a run log into JSON/CSV fields.")
    p.add_argument("log", type=str, help="Path to .log file (or '-' for stdin).")
    p.add_argument("--format", choices=["json", "csv"], default="json")
    p.add_argument("--run-id", type=str, default=None, help="Override run_id (default: filename stem).")
    args = p.parse_args()

    if args.log == "-":
        text = sys.stdin.read()
        run_id = args.run_id or "stdin"
    else:
        path = Path(args.log)
        text = path.read_text(encoding="utf-8", errors="replace")
        run_id = args.run_id or path.stem

    summary = parse_log(text, run_id=run_id)

    if args.format == "json":
        sys.stdout.write(json.dumps(asdict(summary), sort_keys=True) + "\n")
        return 0

    # csv
    w = csv.DictWriter(
        sys.stdout,
        fieldnames=["run_id", "hit", "hit_threshold_m", "min_miss_m", "layer_at_hit", "layer_last"],
    )
    w.writeheader()
    w.writerow(asdict(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

