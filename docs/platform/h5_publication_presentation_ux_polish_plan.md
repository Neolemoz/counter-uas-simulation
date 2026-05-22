# PHASE H5 — Publication / Presentation UX Polish (PLAT-SA-H5)

**Status:** implementation wave  
**Depends on:** PLAN-SA-H1, PLAT-SA-H2, PLAT-SA-H3, PLAT-SA-H4, PLAT-SA-E1 (frozen)

## Purpose

Apply viewer-only visual and presentation polish so the sandbox reads as a **calm research presentation environment** — not a tactical console — while preserving replay-only, explanatory, deterministic semantics.

## Allowed

- Design tokens, Tailwind extend, `@layer` component classes, print CSS
- Presentation/compare/report visual refinement
- Map `visualProfile` publication tuning (render-only)
- Print handoff and Cesium canvas PNG snapshot (no new npm deps)
- Clipboard chapter summary, markdown export links
- Governance lint additive patterns for UI copy
- Bounded `useSegmentPanels` wiring in discover rails

## Forbidden

- Live ROS/WebSocket/rosbridge, browser-triggered execution
- HITL/C2/tactical/readiness/deployment/ML recommendation UX
- Game/HUD aesthetics, blinking live-sensor semantics
- Parser/topic/schema changes
- New required bundle fields
- html2canvas or heavy export dependencies

## Deliverables

| ID | Artifact |
|----|----------|
| D0 | This plan, continuity notes, freeze audit |
| D1 | `sandboxTheme.ts`, `index.css`, `CollapsiblePanelSection` |
| D2 | GovernanceChrome, SegmentNav, WorkspaceShell, App loader |
| D3 | Presentation stack polish |
| D4 | Compare, timeline, Report cards |
| D5 | Cesium `visualProfile` publication |
| D6 | `exportPublicationFrame.ts` + control wiring |
| D7 | Discover rail styling + `useSegmentPanels` |
| D8 | governance_lint_sa extension |

## Post-H5 boundary (not H5)

- H3/H4 execution lane (async workers, CI capture automation)
- PLAN-VIZ-R2 figure merge, chapter thumbnails, localStorage collapse
- Concept B focus layout, authoring workstation, browser orchestration

*End of PLAT-SA-H5 plan.*
