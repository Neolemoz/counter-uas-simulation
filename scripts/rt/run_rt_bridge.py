#!/usr/bin/env python3
"""Start local RT sandbox bridge (loopback HTTP only)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_BRIDGE_PKG = _REPO / "platform" / "rt-sandbox-bridge"
if str(_BRIDGE_PKG) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_PKG))

from rt_sandbox.bridge_server import DEFAULT_HOST, DEFAULT_PORT, serve_forever  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="RT sandbox bridge (PLAT-RT-S2 prototype)")
    parser.add_argument("--host", default=DEFAULT_HOST, help="bind host (loopback only)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="bind port")
    parser.add_argument(
        "--maintainer-smoke-rates",
        action="store_true",
        help="relaxed command rate limits for maintainer live-smoke harness",
    )
    args = parser.parse_args()
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print("error: host must be loopback", file=sys.stderr)
        return 2
    manager = None
    if args.maintainer_smoke_rates:
        from rt_sandbox.governance import GovernanceConfig  # noqa: E402
        from rt_sandbox.session_manager import BridgeSessionManager  # noqa: E402

        manager = BridgeSessionManager(
            config=GovernanceConfig(
                command_rate_burst=1000,
                command_rate_sustained=1000.0,
            ),
            repo_root=_REPO,
        )
    print(f"RT bridge listening on http://{args.host}:{args.port}/v1/command")
    serve_forever(host=args.host, port=args.port, repo_root=_REPO, manager=manager)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
