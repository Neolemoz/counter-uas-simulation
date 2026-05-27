# RT-F2 — Technical Debt Refresh R1

**Phase:** PLAN-RT-F2  
**Baseline:** [rt_r2_technical_debt_audit_r1.md](rt_r2_technical_debt_audit_r1.md)

## Addressed by PLAT-RT-F2 (planned)

| R2 residual | F2 action |
|-------------|-----------|
| `session.tactical` not cleared on teardown | `clear_tactical_state` |
| Experiment import throws / corrupt cache wipe | `experimentImportGuards`, resilient `readCache` |
| Annex cache orphan keys | `pruneAnnexCacheForManifest` |
| `editBySession` orphan keys | App prune effect |
| `shortId` duplicate | `experimentIds.ts` |

## Deferred

| Item | Notes |
|------|-------|
| `App.tsx` decomposition | Still ~600+ LOC; future maintenance wave |
| Live Gazebo CI smoke | Out of F2 scope |
| Playwright E2E | Out of F2 scope |

**Verdict:** F2 closes targeted R2 maintenance gaps without expansion.
