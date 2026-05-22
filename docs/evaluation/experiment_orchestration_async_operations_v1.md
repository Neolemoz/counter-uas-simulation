# Experiment Orchestration Async Operations (v1)

**Phase:** PLAT-SA-I2 — async orchestration foundations  
**Authority:** [AGENTS.md](../../AGENTS.md)

CLI-maintainer commands for async bookkeeping, integrity audits, and replay reproducibility checks. **Viewer observes only.**

---

## Commands

```bash
# Initialize async sidecar for a manifest
python3 scripts/evaluation/record_async_execution.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json --init-async

# Record claim + worker provenance (dry-run bookkeeping)
python3 scripts/evaluation/record_async_execution.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json \
  --claim --worker-id cli-worker-ridge-01 --dry-run

# Compute and store execution fingerprint
python3 scripts/evaluation/record_async_execution.py \
  fixtures/orchestration/manifests/ridge_defense_synthetic.json --execution-fingerprint

# Corpus async integrity audit
python3 scripts/evaluation/audit_orchestration_async_integrity.py --strict

# Lint async sidecars
python3 scripts/evaluation/lint_orchestration_async_manifest.py --all-manifests --check

# Sync viewer mirrors (includes async artifacts)
python3 scripts/evaluation/sync_orchestration_mirrors.py
```

---

## Flags (H3 runner)

| Flag | Default | Purpose |
|------|---------|---------|
| `--allow-runtime-capture` | off | Allow `runtime_capture` steps (frozen H3) |
| `--allow-async-worker` | off | Allow async worker metadata attachment (PLAT-SA-I2) |

Neither flag is enabled in `tier0-sa-r0`.

---

## Related

- [experiment_orchestration_async_manifest_v1.md](experiment_orchestration_async_manifest_v1.md)
- [experiment_orchestration_operations_v1.md](experiment_orchestration_operations_v1.md)

*End of experiment orchestration async operations v1.*
