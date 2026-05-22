#!/usr/bin/env python3
"""Assert scenario catalog, compare pairs, and sweeps index are synced to viewer public demo."""

from __future__ import annotations

import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from audit_sa_platform_integrity import check_catalog_sync  # noqa: E402


def main() -> None:
    errors = check_catalog_sync()
    if errors:
        for err in errors:
            print(err, file=sys.stderr)
        raise SystemExit(1)
    print("SA catalog sync OK")


if __name__ == "__main__":
    main()
