# RT — Post-X3 Technical Debt Audit R1 (CHECKPOINT-RT-POST-X3)

**Phase:** CHECKPOINT-RT-POST-X3
**Status:** frozen audit
**Baseline:** `4d85865`

## Priority summary

| Priority | Items |
|----------|-------|
| **P0** | None blocking — platform shippable for local maintainer workstation |
| **P1** | `ExperimentWorkbenchPanel` concentration (~672 LOC); bundle/code-split before next large UI wave |
| **Advisory** | Shared cognition strip row primitives; further experiment parent splits; bridge SA scan false positives |

## Debt inventory

| Debt | Evidence | Risk | Priority |
|------|----------|------|----------|
| `App.tsx` orchestration | ~280 LOC post-C4 (was ~777 at POST-V4) | Low-Med | Resolved for now |
| `ExperimentWorkbenchPanel` concentration | ~672 LOC; manifest, metrics, review v2/v3, compare, handoff effects | Medium | **P1** |
| `experiment/` module count | ~129 TS/TSX files; good helper factoring (X3 added focused modules) | Low-Med | Advisory |
| `workstation/` | Session tabs, workflow strip, slots; reasonable size | Low | Healthy |
| Compare/cognition duplication | Partially unified in X3; strips still numerous | Low-Med | Advisory |
| Bundle size | 558.12 kB minified / 152.93 kB gzip; Vite >500 kB warning | Medium | **P1** (defer until pain) |
| Bridge pytest | 151 pass, 2 fail on SA path literal scans | Low | Advisory (pre-existing) |

## Non-debt / healthy areas

- Bridge/runtime contracts stable; no X3 bridge diff.
- C4 App decomposition delivered measurable LOC reduction.
- V4 visualization modules separable and frozen.
- X3 export guard tests enforce no `sections[]` in download/copy JSON.
- Validation confidence acceptable (`tier0-rt-ui` OK at X3 P2).

## Cleanup priority (advisory only)

1. **Pause** — preserve stability after V4+C4+X3 (recommended).
2. **PLAN-RT-F9** — if advisory/triage maintainer pain is binding (high governance cost).
3. **PLAN-RT-V5** — if bundle/density/visual compare pain dominates (Cesium coupling).
4. **Experiment parent split** — only with scoped PLAT plan if edit friction returns.

No technical debt cleanup is implemented or authorized by this audit.
