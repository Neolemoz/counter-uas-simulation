# SA Platform Maturity Assessment R1 (G1)

**Phase:** G1 — Platform Governance & Frontier Review  
**Checkpoint:** `75ea6d5` (PLAT-SA-F1a–F1d frozen)  
**Companion:** [sa_platform_governance_review_r1.md](sa_platform_governance_review_r1.md), [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md)

**Build recommendation:** no-build. Assessment only.

---

## 1. Executive summary

The SA platform has reached a **maturity plateau**: a coherent, replay-first, offline reviewer workstation with deterministic corpus operations, integrity gates, and publication/export paths. Architectural coherence is **strong within the platform frontier**; primary risks are **organizational** (regen complexity, documentation duplication, misreading derived artifacts) rather than missing core capabilities.

| Dimension | Maturity (1–5) | Trend |
|-----------|----------------|-------|
| Architectural coherence | 4 | Stable |
| Replay-first consistency | 5 | Stable |
| Determinism guarantees | 4 | Stable (scripted; not bitwise runtime) |
| Governance hardening | 4 | Improving (STAB + F1) |
| Maintainer usability | 3 | Needs doc/regen discipline |
| Reviewer usability | 4 | Strong for demo path |
| Corpus scalability | 3 | Single corpus; paths exist for r2 |
| Publication workflow | 4 | Static offline mature |
| CI / integrity | 4 | tier0-sa-r0 + 22 checks |
| Technical debt | 3 | Regen chain, doc overlap |

**Overall:** Mature enough for **maintenance mode** and selective planning waves; **not** mature enough for unfrozen feature expansion without renewed freeze discipline.

---

## 2. Architectural coherence

### Strengths

1. **Clear stack layering:** R0 bundle → B/C scenarios → D compare/MC → D3 narrative → E1/E2 presentation/synthesis → F1 corpus — each wave has plan + freeze audit + registry row.
2. **Consistent artifact naming:** `*_v1` schemas, `build_*` / `export_*` / `gen_*` conventions, Zod in viewer for JSON contracts.
3. **Separation of concerns:** Linkage (similarity) vs lineage (derivation) documented in F1a audit; evolution vs operational lifecycle distinguished in F1d.
4. **Integrity as platform spine:** PLAT-SA-STAB centralizes parity; F1 extended checks without replacing STAB model.
5. **Three-tier artifact model:** Source fixtures, viewer mirrors, research bundle — reduces ad hoc `cp`.

### Weaknesses

1. **Wave ID proliferation:** R0, B1, B2, C1a, C1b, D1–D3, E1–E2, STAB, F1a–F1d — onboarding requires registry, not one diagram in README.
2. **Dual documentation paths:** Evaluation README, SA plans, corpus schema docs, and freeze audits overlap.
3. **Viewer + script coupling:** Panel features assume specific JSON paths; refactors require synchronized regen (mitigated by audits, not by loose coupling).
4. **Pre-SA and SA coexistence:** EVAL-* tooling remains parallel entry point; correct but increases surface area.

### Coherence verdict

**Coherent** for maintainers following freeze registry and maintainer checklist. **Fragile** for casual contributors who skip regen order or conflate mirrors with source.

---

## 3. Replay-first consistency

| Check | Status | Evidence |
|-------|--------|----------|
| No live ROS in SA viewer | Pass | Static Cesium + fetch JSON |
| Bundles as viewer input | Pass | `replay_sa_bundle_v1` |
| MC/sweeps offline | Pass | `replay_mc_sweep_v1`, committed fixtures |
| Compare mode static | Pass | D1 freeze audit |
| Corpus browser offline | Pass | F1c; deep links query params only |
| Regen CLI-side | Pass | F1b orchestrator; viewer does not execute |

Replay-first alignment: **exemplary** relative to stated AGENTS.md goals. Remaining risk is **language** in fixtures (“dashboard”, “validation”) — governed by lint, not architecture.

---

## 4. Determinism guarantees

### What is deterministic

- Script-generated JSON/MD with `--check` stale detection
- `verify_replay_corpus_reproducibility.py` for corpus regen
- Matched-seed MC discipline (comparability, not bitwise identity)
- Integrity audit allowlists for synthesis/report parity
- Index build from fixture tree (`replay_corpus_index_v1`, 57 entries at checkpoint)

### What is explicitly not guaranteed

- Bitwise Gazebo rerun identity (documented in evaluation README)
- PNG byte parity on every CI run (maintainer regen)
- Duplicate narrative bullets (warnings only)
- Rule-based evolution narratives (not causal inference)

### Determinism maturity

**Strong for artifact regeneration**; **honest about runtime limits**. Suitable for offline research bundles and release snapshots (`sa_r0_corpus_r1_r1`).

---

## 5. Governance hardening maturity

| Mechanism | Maturity | Notes |
|-----------|----------|-------|
| AGENTS.md | Mature | Primary authority |
| Freeze registry | Mature | PLAT-SA-R0–F1d indexed |
| Per-wave freeze audits | Mature | 15+ SA audits |
| Reviewer interpretation guide | Mature | Layer map + glossary |
| `governance_lint_sa.py` | Good | Fixture copy |
| `audit_sa_platform_integrity.py` | Good | 22 checks at F1 gate |
| `tier0-sa-r0` CI | Good | pytest + viewer + audit |
| Meta-governance review | Good | META-GOV-R1; pre-F1 |
| Demo workflow R1 | Good | Docs frozen |

**Gap:** G1 itself is now the post-F1 consolidation artifact; registry should reference these three G1 docs under related planning (index update optional).

---

## 6. Subsystem inventory

### Stable / frozen

| Category | Members |
|----------|---------|
| Platform implementation | PLAT-SA-R0 through F1d, STAB |
| Evaluation replay core | EVAL-RO, EVAL-RI, EVAL-RN-T1, EVAL-RN-V3, EVAL-VIZ-R1/C1/UX-R2 |
| Docs frozen | EVAL-DEMO-R1, PLAN-SA-R1, registry |
| Runtime realism | RT-1..7 (separate frontier) |

### Experimental / planning only

| Item | Status |
|------|--------|
| PLAN-VIZ-R2 | Planning; not implemented |
| Multi-corpus federation | Deferred in F1a limitations |
| Second release `r2` | Optional maintainer action |
| `corpus_ref` full backfill | Optional hygiene |

### Technical debt

| Debt | Impact | Remediation class |
|------|--------|-------------------|
| Long regen chain | Maintainer friction | Docs only; optional orchestrator UX (no new semantics) |
| Info-level drift findings | Noise in audits | `corpus_ref` backfill |
| unindexed meta-corpus files | Index completeness | Optional index extension wave |
| Duplicate caveat prose | Doc drift | Link to reviewer guide |
| 64 evaluation scripts | Discovery | Registry / README index (exists; keep current) |
| ~414 fixture files | CI time / repo size | Policy for sweep growth before expansion |
| ~1789 viewer LOC | Panel coupling | Freeze viewer features |

### Documentation debt

- F1 added 10+ schema/workflow docs — high quality but high volume
- AGENTS.md frontier list is long — registry is the navigation aid
- Cross-link G1 reviews from maintainer checklist (recommended hygiene)
- PLAN-SA-H1 sandbox UX architecture plan addresses viewer presentation debt (plan-only): [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md)

---

## 7. Reviewer usability maturity

| Workflow | Maturity | Artifacts |
|----------|----------|-----------|
| 15-minute demo path | High | [sa_r0_reviewer_quickstart.md](sa_r0_reviewer_quickstart.md) |
| Sweep workstation | High | D3 filmstrip, patterns, exports |
| Compare / topology | High | D1 panels |
| Presentation / storyboard | High | E1 |
| Research bundle portable | High | E2 export |
| Corpus navigation | High | F1c browser, `?corpus_entry=` |
| Long-horizon chronology | Good | F1d evolution panel |
| Maintainer regen | Medium | Checklist exists; steep for new users |

Reviewer UX is **mature for guided review**; **not** designed for untrained operators or live missions (correct per governance).

---

## 8. Corpus scalability limits

| Limit | Current state | Breaking point |
|-------|---------------|----------------|
| Single corpus ID | `sa_r0_corpus_r1` | Multi-team corpora need registry wave |
| Index entries | 57 at checkpoint | Hundreds OK with static JSON; thousands need pagination design |
| Release snapshots | One frozen `r1_r1` | `r2` path exists |
| Full-tree copies | Release = static snapshot | Disk + regen time grow linearly |
| Lineage DAG depth | Shallow on fixtures | Deep DAG needs orphan quarantine (F1b noted) |
| Drift report | Info + warn levels | Alert fatigue if fixtures churn without regen |

Corpus ops are **production-quality for a single research corpus**; **not** proven for multi-tenant or high-churn continuous integration of fixtures.

---

## 9. Publication workflow maturity

| Capability | Status |
|------------|--------|
| Per-sweep publication HTML/MD | Frozen E1/E2 |
| Presentation packets | `export_presentation_pack.py` |
| Research bundle | `export_research_bundle.py` |
| Cross-sweep synthesis | `cross_sweep_synthesis_v1` |
| Corpus publication packet | F1d `build_replay_corpus_publication.py` |
| Offline zip archive | `export_replay_corpus_release.py` |
| CDN / live publish | Explicitly out of scope |

Publication path is **mature for offline mentor/research sharing**; **not** a publishing platform as a service.

---

## 10. CI and integrity bottlenecks

**Gate stack (checkpoint):**

- 221 pytest (counter_uas)
- 44 viewer vitest
- `scripts/ci_eval.sh tier0-sa-r0`
- `audit_sa_platform_integrity.py --all` (22 checks)
- F1 evolution/publication/release verify

**Bottlenecks:**

1. Viewer `npm ci` + build on every tier0 run — necessary for contract tests
2. Full `--all` audit on large fixture tree — acceptable today; watch growth
3. Matplotlib/asset regen — intentionally not in CI every run
4. Narrative duplicate warnings — policy: warn not fail

**Integrity maturity:** **Strong gate** for merge; **maintainer-dependent** for visual freshness.

---

## 11. Sustainability observations

### Sustainable if

- New work stays in narrow freeze waves with audits
- Registry updated per freeze
- Regen order preserved
- Platform and runtime PRs stay separated
- Reviewer copy stays lint-clean

### Unsustainable if

- Viewer gains features without freeze audits
- Corpus grows without index/release policy
- Derived panels imply scoring or readiness
- Multiple frontiers land in one branch
- Documentation copies full wave narratives instead of linking

### Long-term mode recommendation

**Research-and-maintenance platform:** continue generating offline corpora and publications for autonomy evaluation research; **do not** pivot to operational deployment stack.

---

## 12. Maturity verdict

| Verdict | Detail |
|---------|--------|
| Platform stack | **Freeze-maintain** (R0–F1d) |
| Expansion | **Pause** until explicit next wave |
| Runtime | **Separate** lifecycle research only |
| Documentation | **Consolidate** (G1 + link discipline) |
| Next maturity lift | Multi-corpus planning OR VIZ-R2 OR regen ergonomics — **one at a time**, docs-first |

See [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md) for ranked frontier candidates and [sa_platform_governance_review_r1.md](sa_platform_governance_review_r1.md) for boundary and roadmap posture.
