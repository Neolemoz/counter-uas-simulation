# RT - Post-V4 Checkpoint Review

**Phase:** Post-V4 checkpoint review
**Status:** docs frozen
**Scope:** Review-only checkpoint after PLAT-RT-F8, PLAT-RT-X2, PLAT-RT-M3, PLAT-RT-V3, and PLAT-RT-V4 completion.

This checkpoint evaluates platform concentration, layer boundaries, UI duplication, validation confidence, performance/bundle posture, and next-frontier ranking. It authorizes no implementation.

## 1. Platform Concentration Review

| Surface | Observation | Checkpoint verdict |
|---------|-------------|--------------------|
| `App.tsx` | 777 lines; owns session workspace wiring, edit state, layer visibility memory, tactical panels, handoff/workbench composition, and shell assembly | **Watchlist** - future edits are feasible but orchestration concentration is high |
| `RuntimeWorkstationShell.tsx` | 80 lines; remains layout-only and low complexity | **Healthy** |
| `ExperimentWorkbenchPanel.tsx` | 831 lines; combines manifest state, import/export prompts, analytics, F5/F5b metrics, v2 review lane, filtering, and handoff rollup effects | **Primary debt candidate** |
| `platform/rt-sandbox-ui/src/experiment/` | Broad but well-factored submodules; panel composition remains concentrated in one parent | **Pass-with-cleanup** |
| V4 visualization modules | Registry, overlay, cognition, and rail logic are separable | **Healthy** |

Conclusion: The platform is stable but edit concentration has shifted from bridge/runtime into UI orchestration. This is acceptable after V4 but should be reviewed before large X3 work.

## 2. Layer Boundary Review

| Layer | Boundary status |
|-------|-----------------|
| Runtime | No checkpoint runtime changes; bridge/session authority remains command truth |
| Replay | Replay/SA imports remain manual/read-only and out of this checkpoint |
| Advisory | F8 advisory surfaces remain advisory, not readiness authority |
| Experiment | X1/X2/F-series experiment surfaces remain derived/review artifacts, not parser or command truth |
| Visualization | V4 remains explanatory/display-only; density budgets warn only |

No V4 drift was found in the reviewed surfaces. The explanatory/authority split remains clear in cognition strips, density summaries, and compare chrome.

## 3. UI Duplication Review

Future cleanup candidates only:

| Pattern | Duplication signal | Candidate cleanup |
|---------|--------------------|-------------------|
| Cognition strips | Several strips repeat title/body/caveat/chip patterns | Shared display primitives for strip rows/chips |
| Toggle rails | Layer toggles, camera buttons, and mode toggles share button styling | Small local button primitive or utility class |
| Compare display | Session compare, experiment compare, fidelity compare, and multi-manifest compare have similar role/status language | Shared compare-status vocabulary helpers |
| Density summaries | Registry rail and cognition hub both summarize density | Keep one helper as source; avoid copy wording in future |
| Experiment imports | Prompt/parse/error flow repeats for manifest/spec/metrics/fidelity metrics | Future import controller hook, no behavior change |

No refactor is authorized by this checkpoint.

## 4. Validation Review

Latest PLAT-RT-V4 P2 validation:

| Suite | Result |
|-------|--------|
| `npm test` | 91 files, 346 passed |
| `npm run build` | pass; JS 533.22 kB / gzip 146.79 kB; existing chunk-size warning |
| `scripts/ci_eval.sh tier0-rt-ui` | OK |
| `lint_rt_runtime_subcommands.py --check` | OK (7 subcommands) |
| `test_rt_sandbox_bridge.py -q` | 151 passed, 2 failed pre-existing SA string-scan failures |
| `git diff --check` | pass |

The two bridge pytest failures remain the documented literal scans for `platform/sa-r0-viewer`; they were not introduced by V4 and do not indicate runtime/bridge regression.

## 5. Performance / Bundle Review

Vite continues to warn that the main JS chunk exceeds 500 kB after minification. This is consistent with the Cesium-heavy RT UI and accumulated workstation modules. Current posture is acceptable for a local maintainer/demo workstation, but future broad UI waves should consider route-level or panel-level code splitting.

Advisory performance risks:

- Cesium + workstation + experiment panels remain bundled in one app path.
- `ExperimentWorkbenchPanel` composes many render-heavy panels under one parent.
- Multiple compare/cognition strips are cheap individually but numerous.
- No evidence of runtime authority or bridge performance regression.

## 6. Roadmap Review

| Rank | Frontier | Coupling risk | Governance cost | Maintainer value | Verdict |
|------|----------|---------------|-----------------|------------------|---------|
| 1 | **Checkpoint cleanup** | Low-Med | Low | High | Best next if maintainers want lower UI edit risk before X3 |
| 2 | **PLAN-RT-X3** | Med-High | Med | Med-High | Valuable only with documented experiment gaps and narrow docs-first scope |
| 3 | **Pause plateau** | Low | Low | Med | Acceptable if no near-term maintainer pain exists |

Recommendation: checkpoint cleanup first if implementation resumes; otherwise PLAN-RT-X3 should remain docs-first and narrow.

## Stop Line

Checkpoint review complete and docs frozen. Do not start cleanup, PLAN-RT-X3, bridge work, SA viewer work, import automation, federation, or distributed runtime without a new scoped plan, governance review, validation, and freeze audit.
