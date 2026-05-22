# SA H5 — Publication / Presentation UX Polish Freeze Audit (PLAT-SA-H5)

## Scope

Viewer-only visual polish per [h5_publication_presentation_ux_polish_plan.md](../platform/h5_publication_presentation_ux_polish_plan.md):

- `platform/sa-r0-viewer/src/theme/sandboxTheme.ts`, `index.css` design layers + print CSS
- Presentation stack (`PresentationView`, controls, chapter stepper, storytelling)
- Compare/report readability, map `visualProfile` publication tuning
- `exportPublicationFrame.ts` (print, map PNG, chapter copy/markdown)
- Governance chrome, segment nav, workspace shell rhythm

No parser/topic/schema changes, no orchestration CLI changes, no live ROS.

## Governance Result

**Verdict: frozen** for PLAT-SA-H5 (presentation UX polish).

## Boundary Checks

| Check | Result |
|-------|--------|
| Authority creep | Pass — export/print are static handoff only |
| Parser safety | Pass — no schema changes |
| Live vs replay | Pass — map snapshot from canvas; no WebSocket |
| Orchestration execution | Pass — unchanged |
| HITL / C2 | Pass — no command UX |
| Tactical / HUD styling | Pass — calm slate/violet research palette |
| Operational copy lint | Pass — `governance_lint_sa` viewer UI scan |

## Visual / presentation improvements

- **Design system** — shared `sandboxTheme` tokens, `sandbox-panel` / button classes, print layout CSS
- **Presentation mode** — grouped toolbar, chapter stepper, progress label, publication map chrome
- **Handoff** — print layout, copy chapter, save map PNG, sweep/bundle markdown links
- **Compare** — slot column headers with scenario titles, calmer timeline markers
- **Report** — storyboard cards with duration hints; print preview entry
- **Map publication profile** — reduced label noise, softer future-track styling
- **Shell** — calmer segment tabs, de-emphasized bundle file loader, improved typography hierarchy

## Validation

```bash
(cd platform/sa-r0-viewer && npm ci && npm test && npm run build)
python3 scripts/evaluation/governance_lint_sa.py
python3 scripts/evaluation/audit_sa_platform_integrity.py --all
scripts/ci_eval.sh tier0-sa-r0
```

## Related

- PLAT-SA-H4 workstation integration (prerequisite)
- PLAT-SA-E1 presentation mode (feature foundation)
- Post-H5: execution lane, PLAN-VIZ-R2, session persistence — separate waves
