#!/usr/bin/env python3
"""Copy fixtures/orchestration snapshots to sa-r0-viewer public/demo (PLAT-SA-H3/I1)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

_EVAL = Path(__file__).resolve().parent
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

import replay_sa_orchestration_integrity as integrity  # noqa: E402
import replay_sa_orchestration_async as orch_async  # noqa: E402
import replay_sa_orchestration_recovery as orch_recovery  # noqa: E402
from experiment_orchestration import load_manifest  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_SRC = _REPO / "fixtures" / "orchestration"
_DST = _REPO / "platform" / "sa-r0-viewer" / "public" / "demo" / "orchestration"


def main() -> None:
    _DST.mkdir(parents=True, exist_ok=True)
    index_entries: list[dict[str, str]] = []

    for sub in (
        "queues",
        "validation_mirrors",
        "manifests",
        "audits",
        "ops",
        "async",
        "claims",
        "workers",
        "recovery",
        "reconciliation",
        "synthesis",
    ):
        src_dir = _SRC / sub
        if not src_dir.is_dir():
            continue
        dst_dir = _DST / sub
        dst_dir.mkdir(parents=True, exist_ok=True)
        glob_pat = "*_ops.json" if sub == "ops" else "*.json"
        for path in sorted(src_dir.glob(glob_pat)):
            shutil.copy2(path, dst_dir / path.name)
            rel = f"orchestration/{sub}/{path.name}"
            entry_id = path.stem
            if sub == "ops" and entry_id.endswith("_ops"):
                entry_id = entry_id.removesuffix("_ops")
            elif sub == "async" and entry_id.endswith("_async"):
                entry_id = entry_id.removesuffix("_async")
            elif sub == "manifests":
                try:
                    entry_id = str(load_manifest(path)["manifest_id"])
                except (json.JSONDecodeError, ValueError, KeyError):
                    entry_id = path.stem
            index_entries.append({"id": entry_id, "kind": sub, "url": f"/demo/{rel}"})

    index = {
        "artifact_type": "orchestration_mirror_index_v1",
        "schema_version": "orchestration_mirror_index_v1",
        "governance_banner": "ORCHESTRATION MIRROR — not live execution state",
        "entries": index_entries,
    }
    (_DST / "index.json").write_text(json.dumps(index, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    report = integrity.run_integrity_audit(strict=False)
    (_DST / "integrity_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    async_report = orch_async.run_async_integrity_audit(strict=False)
    (_DST / "async_integrity_report.json").write_text(
        json.dumps(async_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    recovery_result = orch_recovery.refresh_recovery_artifacts(strict=False)
    recovery_report = recovery_result["recovery"]
    (_DST / "recovery_integrity_report.json").write_text(
        json.dumps(recovery_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    batch = recovery_result["batch"]
    (_DST / "synthesis" / "async_batch_audit_v1.json").parent.mkdir(parents=True, exist_ok=True)
    (_DST / "synthesis" / "async_batch_audit_v1.json").write_text(
        json.dumps(batch, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"sync_orchestration_mirrors: {len(index_entries)} files -> {_DST}")
    print(f"sync_orchestration_mirrors: integrity_report ok={report.get('ok')}")
    print(f"sync_orchestration_mirrors: async_integrity_report ok={async_report.get('ok')}")
    print(f"sync_orchestration_mirrors: recovery_integrity ok={recovery_report.get('ok')}")


if __name__ == "__main__":
    main()
