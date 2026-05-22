#!/usr/bin/env python3
"""Copy scenario authoring manifests to sa-r0-viewer public/demo (PLAT-SA-A1/A2)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_authoring_integrity as integrity  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_SRC = _REPO / "fixtures" / "scenarios"
_DST = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "authoring"


def main() -> None:
    _DST.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, str]] = []

    for path in sorted(_SRC.glob("*/authoring_manifest.json")):
        pack_id = path.parent.name
        shutil.copy2(path, _DST / f"{pack_id}.json")
        entries.append({"pack_id": pack_id, "url": f"/demo/authoring/{pack_id}.json"})

    index = {
        "artifact_type": "scenario_authoring_mirror_index_v1",
        "schema_version": "1",
        "governance_banner": "AUTHORING MIRROR — CLI promotes; not live configuration",
        "entries": entries,
    }
    (_DST / "index.json").write_text(
        json.dumps(index, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    report = integrity.run_integrity_audit(strict=False)
    (_DST / "integrity_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"sync_authoring_mirrors: {len(entries)} manifests -> {_DST}")
    print(f"sync_authoring_mirrors: integrity_report ok={report.get('ok')}")


if __name__ == "__main__":
    main()
