# SA A1 Authoring Governance Review R1 (PLAN-SA-A1)

**Phase:** PLAN-SA-A1 — Authoring Workstation Foundations (pre-implementation)  
**Checkpoint:** `75ea6d5` (post PLAT-SA-H5, G1 consolidation)  
**Build recommendation:** **plan-only**. Documentation and governance assessment only. Does not authorize viewer, CLI, fixture, or schema implementation.

**Authority:** [AGENTS.md](../../AGENTS.md) remains primary governance. This review assesses proposed PLAN-SA-A1 scope; it does not replace freeze audits for frozen PLAT-SA-* waves.

**Companion deliverables:**

- [sa_a1_authoring_workstation_foundations_plan.md](../platform/sa_a1_authoring_workstation_foundations_plan.md) — PLAN-SA-A1 master plan
- [scenario_authoring_workflow_v1.md](scenario_authoring_workflow_v1.md), [scenario_authoring_manifest_v1.md](scenario_authoring_manifest_v1.md), [scenario_lineage_provenance_v1.md](scenario_lineage_provenance_v1.md)
- [authoring_workflow_continuity_v1.md](authoring_workflow_continuity_v1.md)
- [sa_a1_authoring_workstation_freeze_audit.md](sa_a1_authoring_workstation_freeze_audit.md) — docs-only freeze audit

---

## 1. Identity fit

### Problem addressed

After PLAT-SA-H1–H5, replay/review/corpus/orchestration cognition is mature. **Authoring cognition** remains procedural: edit topology JSON → validate → optional catalog sync → optional orchestration manifest. Maintainers lack a documented promotion ladder, stale-validation policy, or AUTHORING workstation profile (H1 §11 deferred from H3).

### Fit with AGENTS.md platform identity

| Pillar | A1 alignment |
|--------|----------------|
| Governance-aware | Freeze plan + audit before PLAT-SA-A1 implementation |
| Replay-first | Authoring artifacts explain fixture topology; no live ROS |
| Deterministic | CLI-only promotion; validation snapshots with hash/mtime policy |
| Explanatory-only | Lineage and promotion mirrors ≠ operational authority |
| Parser-safe | No `scenario_topology_v1` required-field changes; sidecar manifest only |
| Additive-only | `authoring_manifest.json` sidecar; optional catalog policy gates |

### Architectural direction preserved

| Layer | Role (unchanged) |
|-------|------------------|
| **Web platform** | Scenario authoring + orchestration + replay/research ecosystem |
| **Gazebo / ROS 2** | Runtime simulation engine only |

A1 does **not** redesign the platform into a live control system, game editor, or military command console.

---

## 2. Boundary matrix

| Check | PLAN-SA-A1 | Notes |
|-------|------------|-------|
| Authority creep | **Pass** | Viewer read-only; CLI promotes |
| Parser safety | **Pass** | Manifest not parser-visible |
| Runtime isolation | **Pass** | No topic/schema/parser changes |
| Live vs replay | **Pass** | No WebSocket/rosbridge/live ROS |
| Browser execution | **Pass** | No `run_experiment_queue`, capture, or catalog sync from UI |
| Operational semantics | **Pass** | No HITL, engage, C2, weapon control |
| Scoring | **Pass** | No readiness, certification, ranking UX |
| Mirrors ≠ authority | **Pass** | Validation/promotion mirrors explanatory |
| Legacy `web/` | **Pass** | Not extended |
| Orchestration execution | **Pass** | Handoff references only; H3 runner unchanged |
| Authoring ≠ editing | **Pass** | No browser topology mutation |
| Promotion ≠ corpus release | **Pass** | Fixture-tier only; F1d gate separate |
| Frontier separation | **Pass** | Not bundled with runtime realism waves |

---

## 3. Overlap and conflict check

### PLAT-SA-C1a / C1b (scenario schema & refinement)

| Topic | C1b state | A1 proposal | Conflict? |
|-------|-----------|-------------|-----------|
| Pack layout | `scenario_topology_v1` frozen | Sidecar `authoring_manifest.json` | **No** — additive |
| `metadata.provenance.baseline_pack_id` | Required for variants | Manifest `parent_pack_id` complements | **No** — documented complement |
| Catalog / picker | `sync_sa_catalog.py` → viewer | Policy: sync after `promoted` | **No** — CLI policy only in PLAT-SA-A1 |
| Provenance panel | Read-only in viewer | Extended via AUTHORING panels | **No** — composition |

### PLAT-SA-H3 (orchestration)

| Topic | H3 state | A1 proposal | Conflict? |
|-------|----------|-------------|-----------|
| `experiment_job_manifest_v1` | Frozen step types | Readiness SHOULD use `promotion_status >= promoted` | **No** — additive lint/policy in impl |
| `validation_mirror` | Per-pack mirror JSON | `validation_snapshot_ref` links to mirror | **No** — cross-reference |
| Deferred promotion | H3 audit deferred to later wave | A1 owns **scenario-pack** promotion | **No** — explicit ownership |

### PLAT-SA-F1 (corpus lineage)

| Topic | F1 state | A1 proposal | Conflict? |
|-------|----------|-------------|-----------|
| Corpus index / release | `corpus_entry_id`, release snapshots | Separate **replay/corpus** plane | **No** — UI copy must not merge DAGs |
| Automated release promotion | G1 P2 deferred | A1 does not subsume | **No** |

### PLAN-SA-H1 (AUTHORING profile)

| Topic | H1 state | A1 proposal | Conflict? |
|-------|----------|-------------|-----------|
| AUTHORING banner | Documented §11 | Specified for `scenario` segment | **No** — fulfills H1 |
| Panel IDs | Stub `discover.validation_status` | `authoring.*` panel registry | **No** — extends registry |

### G1 frontier candidates

| Candidate | A1 interaction |
|-----------|----------------|
| Automated **corpus** release promotion (P2) | **Out of scope** — do not conflate with pack promotion |
| Regen orchestrator ergonomics (P2) | Compatible — CLI helpers only |
| Live SA dashboard (P0) | **Forbidden** |

---

## 4. Risk register

| Risk | Level | Mitigation |
|------|-------|------------|
| Game-editor drift (drag/drop, inline edit) | High | Explicit non-goals in plan §4; freeze audit |
| Authority creep (viewer promotes packs) | High | “Viewer observes; CLI promotes” in workflow doc |
| Stale mirror trust | Med | Deterministic stale-validation downgrade rule |
| Promotion vs corpus release confusion | Med | Separate terminology in lineage doc |
| Duplicate lineage UX (F1 vs pack) | Med | Three-plane model; distinct panel labels |
| Feature smuggling in hygiene PRs | Med | PLAT-SA-A1 requires own audit after docs freeze |

---

## 5. Recommended PLAT-SA-A1 phasing (post docs freeze)

| Phase | Deliverable | Validation |
|-------|-------------|------------|
| 1 | `promote_scenario_pack.py`, manifest lint, stale-validation helper | pytest `test_replay_sa_scenario.py` |
| 2 | Example `authoring_manifest.json` for 2–3 valley variants | `governance_lint_sa.py` |
| 3 | `sync_authoring_mirrors.py` (or extend orchestration sync) | Mirror paths under `public/demo/` |
| 4 | Viewer AUTHORING banner + panels | vitest, `npm run build` |
| 5 | Update freeze audit → `PLAT-SA-A1` **frozen** | `tier0-sa-r0`, integrity audit |

---

## 6. Verdict

**Proceed to PLAN-SA-A1 documentation freeze**, then open **PLAT-SA-A1** as a scoped implementation wave.

| Item | Decision |
|------|----------|
| Platform frontier fit | **Approved** — closes documented asymmetry |
| Governance compatibility | **Pass** — additive, CLI-authoritative, parser-safe |
| Runtime frontier bundling | **Forbidden** |
| Browser topology editing | **Forbidden** |
| Corpus auto-release promotion | **Deferred** — not part of A1 |

**Forbidden without new governance wave:** browser mutation, live orchestration, HITL semantics, parser/topic changes, readiness scoring, ML recommendations, collaborative editing.

---

## 7. Related

- [freeze_registry_r1.md](freeze_registry_r1.md) — PLAN-SA-A1 row (docs frozen)
- [h1_sandbox_ux_architecture_plan.md](../platform/h1_sandbox_ux_architecture_plan.md) — §11 replay vs authoring
- [sa_h3_offline_orchestration_freeze_audit.md](sa_h3_offline_orchestration_freeze_audit.md) — deferred promotion
- [sa_platform_frontier_review_r1.md](sa_platform_frontier_review_r1.md) — G1 candidate matrix

*End of SA A1 Authoring Governance Review R1.*
