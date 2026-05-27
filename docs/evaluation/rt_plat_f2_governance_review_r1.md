# RT-F2 — Governance Review R1 (PLAT-RT-F2)

**Phase:** PLAT-RT-F2 — platform hardening  
**Status:** frozen

## Isolation checklist

| Check | Result |
|-------|--------|
| No SA viewer imports in `rt-sandbox-ui` | Pass |
| No browser `capture_session` in experiment modules | Pass |
| Import guards avoid forbidden lexicon | Pass |
| No new user-facing governance banner (banner sprawl avoided) | Pass |
| Bridge protocol unchanged | Pass |
| Read-only staging audit only (no deletion) | Pass |

## Authority boundaries

- Tactical cleanup is bridge audit only — not parser/replay authority.
- Experiment import guards are UI-local — malformed input does not throw to React root.
- Annex cache prune aligns with manifest runs — mirrors remain non-authoritative.

## Recommendation

Freeze PLAT-RT-F2.
