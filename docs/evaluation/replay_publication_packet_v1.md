# Replay Publication Packet (`replay_publication_packet_v1`)

Print-optimized, publication-grade replay review artifacts for mentor demos and research presentations.

See also: [replay_presentation_v1.md](replay_presentation_v1.md), [sa_e1_research_presentation_plan.md](sa_e1_research_presentation_plan.md).

## Artifact placement

| File | Notes |
|------|-------|
| `publication_packet.html` | Print-optimized HTML per sweep |
| `replay_publication_report_v1.json` | Structured export mirror |
| `cross_sweep_publication_report.md` | Corpus-level mentor composite |

## HTML packet requirements

- Governance banner (non-authoritative notice)
- `@media print` CSS (page breaks, figure sizing)
- Numbered figures (`Figure 1`, `Figure 2`, …)
- Citation-style replay refs: `[Replay: sweep_id/member_id]`
- Chapter refs from `presentation_walkthrough.steps`
- Optional cross-sweep appendix when synthesis JSON present

## JSON report fields

| Field | Required | Notes |
|-------|----------|-------|
| `artifact_type` | yes | `replay_publication_report_v1` |
| `schema_version` | yes | `replay_publication_report_v1` |
| `sweep_id` | yes | Target sweep |
| `figures` | no | `{ figure_id, label, path, caption }[]` |
| `citations` | no | Replay citation entries |
| `chapters` | no | Chapter reference list |
| `cross_sweep_appendix` | no | Synthesis summary pointer |
| `corpus_ref` | no | F1a corpus index pointer for `publication_packet` entry |

## Governance

**Do:** Use relative figure paths in committed fixtures; optional base64 for portable zip.

**Don't:** Imply live monitoring, deployment readiness, or tactical superiority styling.

## Producer / consumer

| Role | Module |
|------|--------|
| Producer | `export_presentation_pack.py`, `replay_publication_html.py` |
| Consumer | Browser print-to-PDF, mentor handoff |
