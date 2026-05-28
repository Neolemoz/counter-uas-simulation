# RT-X2 P2 — Handoff Contamination Review R1

**Phase:** PLAT-RT-X2 P2  
**Baseline:** [rt_advisory_contamination_gates_v1.md](rt_advisory_contamination_gates_v1.md)

## P2 contamination matrix

| ID | Landmine | P2 mitigation | Residual |
|----|----------|---------------|----------|
| X2-CONT-P2-01 | File download implies SA import | Governance banner + filename advisory | **Low** |
| X2-CONT-P2-02 | Multi-manifest diff as operational truth | Metadata-only banner + no run pairing | **Low** |
| X2-CONT-P2-03 | Packet path refs trigger auto handoff | Paths explanatory; no import CLI | **Low** |

## Deny list (P2 UI)

1. `downloadReviewPacket` does not invoke SA import or bridge capture
2. Diff table does not merge manifests or pair `run_id` across experiments
3. No new subprocess / maintainer batch buttons in v2 zone

## Verdict

**Pass** — PLAT-RT-X2 P2 may freeze.
