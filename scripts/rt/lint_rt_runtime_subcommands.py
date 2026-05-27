#!/usr/bin/env python3
"""Lint runtime subcommand registry vs handler coverage (PLAT-RT-R3c)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_RT_SANDBOX = _REPO / "platform/rt-sandbox-bridge"
if str(_RT_SANDBOX) not in sys.path:
    sys.path.insert(0, str(_RT_SANDBOX))

from rt_sandbox.governance import RUNTIME_SUBCOMMANDS  # noqa: E402
from rt_sandbox.runtime_subcommand_governance import (  # noqa: E402
    discover_runtime_subcommand_handlers,
    lint_runtime_subcommands,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Lint RUNTIME_SUBCOMMANDS vs session_runtime_commands handlers."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit 1 when drift is detected (CI mode).",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit machine-readable report on stdout.",
    )
    args = parser.parse_args()

    issues = lint_runtime_subcommands()
    handlers = discover_runtime_subcommand_handlers()

    if args.json:
        print(
            json.dumps(
                {
                    "ok": not issues,
                    "declared": sorted(RUNTIME_SUBCOMMANDS),
                    "implemented": sorted(handlers),
                    "issues": issues,
                },
                indent=2,
            )
        )
    elif issues:
        for item in issues:
            print(f"lint_rt_runtime_subcommands: {item}", file=sys.stderr)
    else:
        n = len(RUNTIME_SUBCOMMANDS)
        print(f"lint_rt_runtime_subcommands OK ({n} subcommands)")

    if args.check and issues:
        raise SystemExit(1)
    if not args.check and issues:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
