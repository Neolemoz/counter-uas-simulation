# SA F2A — Multi-Corpus Federation Governance Review R1

Plan: [sa_f2a_multi_corpus_federation_plan.md](sa_f2a_multi_corpus_federation_plan.md)

Freeze audit: [sa_f2a_multi_corpus_federation_freeze_audit.md](sa_f2a_multi_corpus_federation_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| Minimal scope? | Yes — offline manifest/index/audit/viewer only |
| Authority creep? | No — viewer read-only; CLI builds fixtures |
| Parser safety? | Yes — no parser/topic changes |
| I3 reopen? | No — additive `corpus_group_id` on reconciliation index only |

## Boundary table

| Allowed | Forbidden |
|---------|-----------|
| Federation manifest registry | Cloud / distributed federation |
| Cross-corpus integrity audits | Live synchronization |
| Read-only federation panels | Collaborative editing |
| Recovery continuity rollup | Browser-triggered orchestration |
| Deterministic snapshots | ML recommendations |

## Recommendation

Proceed with maintenance-only changes after freeze (mirror sync, doc typos). Do **not** start collaborative or cloud federation without a new scoped wave and freeze audit.
