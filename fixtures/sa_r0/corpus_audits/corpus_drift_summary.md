# Corpus drift summary

Corpus drift report for maintainer integrity review only — explanatory fixture drift inventory, not operational failure or readiness.

Corpus: `sa_r0_corpus_r1` · index revision `cf644076b570…`

Total findings: **9**

## By kind

- `corpus_ref_missing`: 4
- `unindexed_file`: 5

- **[info]** `corpus_ref_missing` — optional corpus_ref absent on fixtures/sa_r0/sweeps/delayed_detection_sweep/sweep.json
- **[info]** `corpus_ref_missing` — optional corpus_ref absent on fixtures/sa_r0/sweeps/ridge_overlap_sweep/sweep.json
- **[info]** `corpus_ref_missing` — optional corpus_ref absent on fixtures/sa_r0/sweeps/saturation_assignment_sweep/sweep.json
- **[info]** `corpus_ref_missing` — optional corpus_ref absent on fixtures/sa_r0/sweeps/valley_sensor_sweep/sweep.json
- **[info]** `unindexed_file` — discoverable artifact not in corpus index: fixtures/sa_r0/synthesis/replay_corpus_evolution_manifest_v1.json
- **[info]** `unindexed_file` — discoverable artifact not in corpus index: fixtures/sa_r0/synthesis/replay_corpus_evolution_summary_v1.json
- **[info]** `unindexed_file` — discoverable artifact not in corpus index: fixtures/sa_r0/synthesis/replay_corpus_index_v1.json
- **[info]** `unindexed_file` — discoverable artifact not in corpus index: fixtures/sa_r0/synthesis/replay_corpus_publication_packet_v1.json
- **[info]** `unindexed_file` — discoverable artifact not in corpus index: fixtures/sa_r0/presentations/index.json

## Interpretation

- Drift is fixture integrity only, not operational failure.
