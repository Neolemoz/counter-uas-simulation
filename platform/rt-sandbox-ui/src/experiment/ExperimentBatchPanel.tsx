import { useMemo, useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { BANNER_EXPERIMENT } from "@/governance/banners";
import type { ExperimentBatchSpec } from "./experimentSchema";
import { manifestExportPath } from "./experimentStore";

export function ExperimentBatchPanel({
  experimentId,
  batchSpec,
  onBatchSpecChange,
}: {
  experimentId: string;
  batchSpec: ExperimentBatchSpec;
  onBatchSpecChange: (spec: ExperimentBatchSpec) => void;
}) {
  const [specPath, setSpecPath] = useState("fixtures/rt_experiments/demo_ab_batch_v1.yaml");

  const cliCommand = useMemo(() => {
    const manifestPath = manifestExportPath(experimentId);
    return [
      "python3 scripts/rt/rt_experiment_batch.py",
      `--spec ${specPath}`,
      `--repo-root .`,
      `--manifest-out ${manifestPath}`,
    ].join(" ");
  }, [experimentId, specPath]);

  const addRun = () => {
    const id = `run-${batchSpec.runs.length + 1}`;
    onBatchSpecChange({
      ...batchSpec,
      runs: [
        ...batchSpec.runs,
        { run_id: id, label: id, dwell_s: batchSpec.default_dwell_s ?? 2 },
      ],
    });
  };

  return (
    <PanelShell title="Experiment batch (maintainer CLI)">
      <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_EXPERIMENT}</p>
      <p className="mb-2 text-xs text-slate-500">
        Browser cannot call <code className="text-slate-400">capture_session</code>. Queue runs
        here, then execute via maintainer CLI on loopback bridge.
      </p>
      <label className="mb-2 block text-xs text-slate-400">
        Batch spec path
        <input
          className="mt-1 w-full rounded border border-slate-700 bg-slate-950 px-2 py-1 font-mono text-xs"
          value={specPath}
          onChange={(e) => setSpecPath(e.target.value)}
        />
      </label>
      <ul className="mb-2 space-y-1 text-xs text-slate-400">
        {batchSpec.runs.map((r) => (
          <li key={r.run_id} className="flex gap-2">
            <span className="font-mono text-slate-300">{r.run_id}</span>
            <span>{r.label}</span>
            <span className="text-slate-600">dwell {r.dwell_s ?? batchSpec.default_dwell_s ?? 2}s</span>
          </li>
        ))}
      </ul>
      <button
        type="button"
        className="mb-3 rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
        onClick={addRun}
      >
        Add queue row
      </button>
      <pre className="overflow-x-auto rounded border border-slate-800 bg-slate-950 p-2 text-[10px] text-emerald-200/90">
        {cliCommand}
      </pre>
      <p className="mt-2 text-[10px] text-slate-600">
        Dry-run: append <code>--dry-run</code>. Requires bridge at 127.0.0.1:18765.
      </p>
    </PanelShell>
  );
}
