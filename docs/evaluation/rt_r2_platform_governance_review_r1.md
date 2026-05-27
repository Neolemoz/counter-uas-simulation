# RT-R2 — Platform Governance Review R1

**Phase:** PLAN-RT-R2 — runtime platform maturity review  
**Plan:** [rt_r2_runtime_platform_maturity_plan.md](../platform/rt_r2_runtime_platform_maturity_plan.md)  
**Master review:** [rt_r2_platform_maturity_review_r1.md](rt_r2_platform_maturity_review_r1.md)  
**Freeze audit:** [rt_r2_platform_maturity_freeze_audit.md](rt_r2_platform_maturity_freeze_audit.md)

## Scope verdict

| Question | Answer |
|----------|--------|
| Docs-only wave? | Yes |
| SA viewer live hooks introduced? | No — SA3 read-only replay panels only |
| Parser/topic changes? | No |
| New bridge commands? | No |
| Default behavior changed? | No |
| Frozen PLAT-RT-* regression? | Re-validated; no semantic drift claimed |

**Recommendation:** Freeze **PLAN-RT-R2** (docs frozen).

---

## 1. Authority boundaries

Re-audit against [rt_runtime_governance_v1.md](rt_runtime_governance_v1.md) and [rt_authority_model_v1.md](rt_authority_model_v1.md).

| Surface | Authoritative for sandbox? | Authoritative for SA replay? |
|---------|---------------------------|------------------------------|
| Entity registry / bridge commands | **Yes** (command truth) | No |
| Pose sync mirror | No — explanatory | No |
| Telemetry mirror / pull buffers | No — explanatory | No |
| Staging capture | No — pending approval | No |
| Normalized manifest | No — export boundary | No |
| SA replay bundle | No | **Yes** (static bundle only) |

| Check | Result |
|-------|--------|
| Mirrors ≠ authority in UI | **Pass** — T1/T4 cognition, frozen banners |
| Tactical controller ≠ operational C2 | **Pass** — TAC governance + forbidden lexicon tests |
| Experiment compare ≠ rankings | **Pass** — X1 `BANNER_EXPERIMENT`, diff badges only |

---

## 2. RT↔SA separation

| Check | Result |
|-------|--------|
| `platform/sa-r0-viewer/` not used for live RT | **Pass** |
| `reject_auto_sa_import` / no packager from bridge | **Pass** |
| `session_id` not SA corpus lineage parent | **Pass** — export_boundary |
| Manual import only (SA1 CLIs) | **Pass** |
| Handoff mirror read-only (SA2) | **Pass** |
| Federation cannot start RT sessions | **Pass** |
| SA3 embeds tactical continuity read-only | **Pass** — no RT live subscription in viewer |

| Finding ID | Verdict |
|------------|---------|
| R2-GOV-SA-01 | Pass — RT writes `runs/rt_sandbox/` only |
| R2-GOV-SA-02 | Pass — capture ≠ import unchanged after X1 |
| R2-GOV-SA-03 | Pass-with-conditions — richer SA workflow remains high-risk if automated |

---

## 3. Replay boundaries

| Boundary | Enforcement |
|----------|-------------|
| RT live state ≠ replay authority | Banners, NOT SA REPLAY AUTHORITY |
| Tactical annex in capture ≠ full SA scrubber | TAC5 + SA3 counts/timeline summary |
| Experiment pinned snapshots ≠ parser contracts | `rt_experiment_manifest_v1` local only |
| Normalized capture ≠ automatic corpus commit | Approval + SA1 manual path |

| Check | Result |
|-------|--------|
| SA3 does not mutate RT sessions | **Pass** |
| X1 does not load SA bundles | **Pass** |
| Replay annotations explanatory only | **Pass** — aligned with AGENTS.md |

---

## 4. Deny-by-default rules

| Gate | Implementation |
|------|----------------|
| Command allow-list | `ALLOWED_COMMANDS` |
| Forbidden operational commands | `RT_FORBIDDEN_COMMANDS` |
| Browser forbidden calls | `isolation.test.ts` — no `capture_session` invoke in experiment modules |
| Tactical forbidden lexicon | `FORBIDDEN_LEXICON` + panel tests |
| ROS topic allow-list | `ros_allowlist.py` when adapter on |
| Runtime subcommands | R3c lint gate |

| Check | Result |
|-------|--------|
| No browser→ROS | **Pass** |
| No rosbridge / legacy web RT | **Pass** |
| No federation_register from RT UI | **Pass** |
| Autonomous loop bounded | **Pass** — TAC4 safety review on record |

---

## 5. Banner and lexicon discipline

| Banner | Status |
|--------|--------|
| T1 primary / transient / NOT SA | **Frozen** — unchanged |
| T2–T5 connected banners | **Frozen** |
| V2 terrain disclaimer | **Additive** — frozen |
| X1 experiment disclaimer | **Additive** — frozen |
| Multi-session (M2) | **Frozen** |

| Check | Result |
|-------|--------|
| No operational readiness language in RT UI tests | **Pass** |
| `BANNER_EXPERIMENT` avoids forbidden lexicon | **Pass** — governance.test.ts |

---

## 6. Governance verdict

| Area | Pass? |
|------|-------|
| Authority | Yes |
| RT↔SA | Yes |
| Replay | Yes |
| Deny-by-default | Yes |
| Lexicon | Yes |

**Pass** — PLAN-RT-R2 suitable for freeze. Platform governance posture is **consistent** with frozen wave audits collectively; no new P0 governance blockers at maturity plateau.

**Stop line:** Post-R2 implementation requires per-wave governance review. **Do not** interpret ranked frontiers as authorization.
