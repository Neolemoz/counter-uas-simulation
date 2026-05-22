# SA Platform Usability Findings (Release Polish)

Documentation-only log from merge/release checkpoint review. **Not a new platform wave.** Findings inform mentor/reviewer workflows; fixes stay additive and replay-only.

Authority: [sa_platform_release_checkpoint.md](sa_platform_release_checkpoint.md), [sa_r0_reviewer_quickstart.md](sa_r0_reviewer_quickstart.md).

## Resolved in release polish

| Finding | Mitigation |
|---------|------------|
| `showcase_cross_sweep_synthesis` orphaned from `presentations/index.json` after `gen_e1` regen | Storyboard restored in `gen_e1_presentation_fixtures.py`; index parity check added to `check_storyboard_urls` |
| Reviewer quickstart used nonexistent compare pair `valley_vs_saturation` | Corrected to `valley_vs_ridge_replay` |
| Cohort URL used shorthand `los_fragmented` | Corrected to `los_fragmented_replay_cohort` |
| Walkthrough valley ingress chapter gap (2 → 4) | Renumbered to contiguous 0–3 in storyboard generator |
| `sweep_presentation_packet.md` lacked packet-level framing | Added combined-export header + contents list in `export_presentation_pack.py` |

## Remaining reviewer friction (documented, not redesigned)

| Area | Observation | Mentor guidance |
|------|-------------|-----------------|
| Compare entry | Two URL modes (`?pair=` vs `?compare=`) | Use curated pairs from [compare_pairs.json](../../platform/sa-r0-viewer/public/demo/compare_pairs.json); ad hoc only when pair not cataloged |
| Walkthrough vs storyboard | Mutually exclusive URL params | Pick `walkthrough=1` **or** `presentation=<id>` per session — see quickstart |
| Synthesis panel | No deep-link URL param | Load any sweep, then open synthesis panel in viewer chrome; offline: `/demo/synthesis/cross_sweep_summary.md` |
| Export review pack | Requires sweep context | Load `?sweep=<id>` before using in-viewer export link |
| Sensor studies | Shared demo log across valley experiment packs | Read [comparison_foundations.md](comparison_foundations.md) § Sensor placement study before comparing |
| Static HTML vs SA-R0 | Two onboarding surfaces | Session 1: static comprehension HTML; Session 2: SA-R0 quickstart (~15 min) |
| Research bundle vs viewer | Different surfaces for same corpus | Viewer = interactive spatial review; zip = offline print-to-PDF handoff |

## Walkthrough / presentation pacing notes

| Deck | Minutes | Pacing note |
|------|---------|-------------|
| `walkthrough_valley_ingress_long` | 20 | Four contiguous chapters on single demo pack — best first SA-R0 storyboard |
| `walkthrough_assignment_instability` | 17 | Two scenes; second scene switches sweep without presentation param |
| `showcase_topology_divergence` | 15 | Sweep + filmstrip; good for topology divergence localization |
| `showcase_cross_sweep_synthesis` | 14 | Four scenes across sweeps; mentor must open synthesis panel manually on scene 1 |
| `showcase_best_narratives` | 18 | Cross-surface tour (sweep → bundle walkthrough → sweep) |
| `showcase_ambiguity_heavy` | 16 | Two sweeps; good after ambiguity_pressure static demo |

## Publication / export readability

- **publication_packet.html** — consistent governance banner across all four sweeps; figures use relative `reports/figures/` paths (viewer static server)
- **sweep_presentation_packet.md** — combined summary + guided steps + topology storytelling; use contents header to navigate
- **guided_walkthrough_report.md** — step-by-step mentor script; preferred for live walkthrough narration
- **Duplicate narrative bullets** across sweeps — intentional synthetic template copy; audit warns only

## Dissemination checklist (mentor handoff)

Before external share:

1. Run [sa_platform_maintainer_checklist.md](sa_platform_maintainer_checklist.md) pre-merge gate
2. Share `publication_packet.html` or research bundle zip — not raw logs unless policy allows
3. Include [reviewer_interpretation_guide.md](reviewer_interpretation_guide.md) link
4. State explicitly: explanatory replay review only, not operational readiness
5. For live demo: `npm run dev` + quickstart URL cheat sheet

## Intentional limitations (unchanged)

- No ML summarization or ranking
- Cross-sweep compare stays descriptive; viewer compare stays 2-slot A/B
- PNG byte parity not CI-gated
- PDF export: manual browser print-to-PDF
- PLAN-VIZ-R2 (rosbag/Plotly): planning-only

## Related

- [replay_demo_review_workflow_r1.md](replay_demo_review_workflow_r1.md)
- [sa_stabilization_freeze_audit.md](sa_stabilization_freeze_audit.md)
