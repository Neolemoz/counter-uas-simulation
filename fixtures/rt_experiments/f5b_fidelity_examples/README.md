# F5b fidelity example fixtures (PLAT-RT-F5b)

Reference artifacts for docs and PLAT-RT-F5b golden tests.

| File | Schema |
|------|--------|
| `manifest_fidelity_golden.json` | `rt_experiment_manifest_v1` (2-run fidelity golden) |
| `truth_snapshot_clear.json` | `rt_fidelity_truth_snapshot_v1` |
| `truth_snapshot_divergent.json` | `rt_fidelity_truth_snapshot_v1` |
| `fidelity_pose_block_example.json` | `rt_fidelity_pose_block_v1` (nested) |
| `fidelity_metrics_report_golden.json` | `rt_experiment_fidelity_metrics_report_v1` |
| `staging/run-clear/` | truth snapshot + normalized manifest for `run-clear` |
| `staging/run-divergent/` | truth snapshot + normalized manifest for `run-divergent` |

Contracts: [rt_runtime_fidelity_coupling_v1.md](../../../docs/evaluation/rt_runtime_fidelity_coupling_v1.md), [rt_experiment_metrics_v1.md](../../../docs/evaluation/rt_experiment_metrics_v1.md) §11.
