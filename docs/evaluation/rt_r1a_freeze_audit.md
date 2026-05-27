# RT-R1a — Runtime Vocabulary & Authority Hardening Freeze Audit (PLAT-RT-R1a)

## Scope

- [rt_r1a_runtime_vocabulary_hardening_plan.md](../platform/rt_r1a_runtime_vocabulary_hardening_plan.md)
- [rt_revision_vocabulary_v1.md](rt_revision_vocabulary_v1.md)
- [rt_authority_model_v1.md](rt_authority_model_v1.md)
- [rt_audit_event_vocabulary_v1.md](rt_audit_event_vocabulary_v1.md)
- `platform/rt-sandbox-bridge/rt_sandbox/authority_labels.py`
- `platform/rt-sandbox-bridge/rt_sandbox/audit_vocabulary.py`
- Extended `audit_log.py`, `export_audit_log.py`, `telemetry_bridge.py`, `capture_normalize.py`
- [rt_r1a_governance_review_r1.md](rt_r1a_governance_review_r1.md)

Not in scope: telemetry UI, Cesium, SA integration, poll unification, session_manager refactor.

Prerequisite: PLAN-RT-R1 frozen; PLAT-RT-G5 frozen.

## Governance Result

**Verdict: frozen** for PLAT-RT-R1a.

## Boundary Checks

| Check | Result |
|-------|--------|
| SA viewer unchanged | Pass |
| Parser/topic safety | Pass |
| Additive-only behavior | Pass |
| P0 roadmap closure | Pass |
| No new bridge commands | Pass |
| Frozen PLAT-RT-* semantics preserved | Pass |

## Deliverable Checklist

| # | Artifact | Frozen |
|---|----------|--------|
| 1 | `rt_revision_vocabulary_v1.md` | Yes |
| 2 | `rt_authority_model_v1.md` | Yes |
| 3 | `rt_audit_event_vocabulary_v1.md` | Yes |
| 4 | `authority_labels.py` | Yes |
| 5 | `audit_vocabulary.py` | Yes |
| 6 | Audit `event_kind` auto-classification | Yes |
| 7 | Telemetry `source` + `authority_label` on all channels | Yes |
| 8 | Normalized manifest `authority_model` block | Yes |
| 9 | Cross-links in sync/telemetry/capture contracts | Yes |
| 10 | Governance review R1 | Yes |
| 11 | Freeze audit (this document) | Yes |
| 12 | Additive R1a tests (4) | Yes |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_rt_sandbox_bridge.py -q
```

88 passed at freeze time (84 prior + 4 R1a).

## Naming / authority improvements

- Single revision glossary for all RT counter names.
- Explicit `source` + `authority_label` on telemetry channel payloads (stub and adapter paths).
- `event_kind` disambiguates audit entries from invokable `command_type` values.
- Normalized captures include `authority_model` legend and per-record authority labels.

## Remaining P1/P2 risks

- Adapter poll unification (R2a), stale UTC helper merge (R2b), telemetry path decision (R2c) — **closed by PLAT-RT-R1b**
- Template adapter resync policy (R2d), session_manager decomposition (R3a)
- See [rt_roadmap_post_g5_v1.md](rt_roadmap_post_g5_v1.md) P1/P2 tables

## Stop Line

Do not start R2a or expansion waves until PLAT-RT-R1a is frozen. PLAT-RT-G5 and post-R1 stop lines unchanged for SA ingestion, federation, and viewer runtime hooks.
