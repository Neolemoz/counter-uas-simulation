# SA B2 Rich Replay Scenario Packs Freeze Audit (PLAT-SA-B2)

## Scope

- Six B2 scenario packs + demos: `multi_ridge`, `corridor_defense`, `saturation_ingress`, `urban_masking`, `delayed_detection`, `long_range_ingress`
- [`scripts/evaluation/gen_b2_scenarios.py`](../../scripts/evaluation/gen_b2_scenarios.py) regen helper
- Additive [`replay_sa_bundle.py`](../../scripts/evaluation/replay_sa_bundle.py) multi-track log parsing
- Additive [`replay_sa_geometry.py`](../../scripts/evaluation/replay_sa_geometry.py) multi-threat LOS budget
- [`fixtures/scenarios/index.json`](../../fixtures/scenarios/index.json) catalog
- Viewer [`DEMO_ALIASES`](../../platform/sa-r0-viewer/src/replay/loadBundle.ts) only

No runtime, parser, topic, governance chrome, or scenario-picker changes.

## Governance Result

**Verdict: frozen** for PLAT-SA-B2.

| Check | Result |
|-------|--------|
| Authority creep | Pass — all packs carry fictional/explanatory governance |
| Parser safety | Pass — evaluation-side fixtures and bundles only |
| Runtime isolation | Pass — synthetic logs, no ROS capture in wave |
| Operational semantics | Pass — no C2/engage/readiness |
| Dual source of truth | Pass — topology packs canonical; demos derived |
| SA-R0 boundary | Pass — additive bundles and aliases |

## Regression Evidence

```bash
python3 -m pytest src/counter_uas/test/test_replay_sa_scenario.py \
  src/counter_uas/test/test_replay_sa_bundle.py \
  src/counter_uas/test/test_replay_sa_geometry.py -q
for d in ridge_defense valley_ingress multi_ridge corridor_defense \
  saturation_ingress urban_masking delayed_detection long_range_ingress; do
  python3 scripts/evaluation/validate_scenario.py "fixtures/scenarios/$d"
done
cd platform/sa-r0-viewer && npm ci && npm test && npm run build
scripts/ci_eval.sh tier0-sa-r0
```

## Replay UX observations (manual review)

| Scenario | Readability | Overlay / LOS | Narrative | Pacing |
|----------|-------------|---------------|-----------|--------|
| multi_ridge | Staggered masks help ridge transitions; LOS count high with 2 radars | 3 ridge + 2 blocked overlays readable with toggles | Reacquisition annotations align with gap window | Medium |
| corridor_defense | Narrow corridor clear; flank blocks add depth | Overlapping north/south LOS segments need layer toggle | Timing-pressure annotations useful | Short / compressed |
| saturation_ingress | Two threat colors distinguishable; assignment story dense | Overlap pocket + dual ingress corridors busy | Ambiguity annotations essential for interpretation | Medium |
| urban_masking | 8 small blocks clutter map at full zoom | Degraded_visibility grid stresses toggle discipline | Continuity/stale annotations clarify gaps | Medium |
| delayed_detection | Late first sample obvious on scrub | Tight zones communicate urgency | Late-acquisition annotation matches line ~15 | Short after detection |
| long_range_ingress | Long trail aids spatial story; launch segments visible | Terrain + corridor scale well at fit-replay | Bookmarks would help (C1b) | Long — scrubber valuable |

**Mock panes:** EoIR/radar/onboard still bind to first threat/interceptor only — secondary tracks visible on map but not in mocks.

## Schema stress points

- Circle-only `zones` cannot represent elongated corridors; `ingress_corridor` polygons carry that story
- `active_t_range` is manual; pack-time lint against log span not implemented (C1b)
- Multi-threat LOS hits `MAX_LOS_SEGMENTS` cap quickly with multiple radars
- Log grammar extension (`threat_id=`, `interceptor_id=`) is additive but undocumented in parser contracts (documented in scenario schema doc)

## C1b recommendations

- Catalog-driven scenario picker wired to `fixtures/scenarios/index.json`
- Pack-time lint: overlay `active_t_range` ⊆ log line span
- Per-threat LOS budget toggle in viewer
- Multi-threat mock pane selection
- Ellipse/line-string zone geometry (schema v2)

## Follow-on

- **C1b:** Scenario selection UX, provenance panel, comparison abstractions
