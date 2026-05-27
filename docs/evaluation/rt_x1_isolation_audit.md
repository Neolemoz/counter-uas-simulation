# RT-X1 — Isolation Audit

**Phase:** PLAT-RT-X1

## UI isolation

| Check | Result |
|-------|--------|
| `platform/rt-sandbox-ui` has no `sa-r0-viewer` imports | Pass |
| Bridge client excludes `capture_session` | Pass |
| Experiment modules under `src/experiment/` | Pass |

## Maintainer path

| Check | Result |
|-------|--------|
| `rt_experiment_batch.py` uses `send_command` only | Pass |
| `capture_session` only in maintainer script | Pass |

## Verdict

**Pass**
