import { useMemo, useState } from "react";
import { PanelShell } from "@/components/GovernanceChrome";
import { BANNER_ANALYTICS } from "@/governance/banners";
import type { ExperimentBatchSpec, SweepGroup } from "./experimentSchema";
import { DEFAULT_SWEEP_CATALOG } from "./sweepCatalogData";
import {
  batchSpecToYaml,
  compileSweepGroup,
  parseSweepCatalog,
  suggestedExperimentId,
} from "./sweepCompile";
import { manifestExportPath } from "./experimentStore";

export function SweepCatalogBrowser({
  experimentId,
  onApplyBatchSpec,
}: {
  experimentId: string;
  onApplyBatchSpec: (spec: ExperimentBatchSpec) => void;
}) {
  const [catalog, setCatalog] = useState(DEFAULT_SWEEP_CATALOG);
  const [groupId, setGroupId] = useState(DEFAULT_SWEEP_CATALOG.sweep_groups[0]?.group_id ?? "");
  const specPath = "fixtures/rt_experiments/sweep_catalog_v1.yaml";

  const group: SweepGroup | undefined = catalog.sweep_groups.find((g) => g.group_id === groupId);

  const compiled = useMemo(() => {
    if (!group) return null;
    const expId = suggestedExperimentId(group.group_id);
    return compileSweepGroup(group, expId);
  }, [group]);

  const cliCommand = useMemo(() => {
    if (!compiled) return "";
    const manifestPath = manifestExportPath(compiled.experiment_id);
    return [
      "python3 scripts/rt/rt_experiment_batch.py",
      `--spec fixtures/rt_experiments/${compiled.experiment_id}.yaml`,
      `--repo-root .`,
      `--manifest-out ${manifestPath}`,
    ].join(" ");
  }, [compiled]);

  const importCatalog = () => {
    const text = window.prompt("Paste rt_experiment_sweep_catalog_v1 JSON");
    if (!text) return;
    try {
      const parsed = parseSweepCatalog(JSON.parse(text));
      setCatalog(parsed);
      setGroupId(parsed.sweep_groups[0]?.group_id ?? "");
    } catch {
      window.alert("Invalid sweep catalog JSON");
    }
  };

  const downloadBatchYaml = () => {
    if (!compiled) return;
    const yaml = batchSpecToYaml(compiled);
    const blob = new Blob([yaml], { type: "text/yaml" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${compiled.experiment_id}.yaml`;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <PanelShell title="Sweep catalog">
      <p className="mb-2 text-[10px] text-amber-100/80">{BANNER_ANALYTICS}</p>
      <p className="mb-2 text-xs text-slate-500">{catalog.governance_banner}</p>
      <p className="mb-2 text-[10px] text-slate-600">
        Source: <code className="text-slate-400">{specPath}</code> (repo fixture)
      </p>
      <div className="mb-2 flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
          onClick={importCatalog}
        >
          Import catalog JSON
        </button>
        {compiled && (
          <>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={() => onApplyBatchSpec({ ...compiled, experiment_id: experimentId })}
            >
              Apply to batch queue
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={downloadBatchYaml}
            >
              Download batch YAML
            </button>
          </>
        )}
      </div>
      <label className="mb-2 block text-xs text-slate-400">
        Sweep group
        <select
          className="mt-1 block w-full rounded border border-slate-700 bg-slate-950 px-2 py-1 text-xs"
          value={groupId}
          onChange={(e) => setGroupId(e.target.value)}
        >
          {catalog.sweep_groups.map((g) => (
            <option key={g.group_id} value={g.group_id}>
              {g.label}
            </option>
          ))}
        </select>
      </label>
      {group && (
        <>
          <p className="mb-2 text-xs text-slate-500">{group.description}</p>
          <table className="mb-3 w-full text-xs">
            <thead>
              <tr className="text-left text-slate-500">
                <th className="pb-1">entry</th>
                <th className="pb-1">template</th>
                <th className="pb-1">dwell</th>
                <th className="pb-1">hint</th>
              </tr>
            </thead>
            <tbody>
              {group.sweep_entries.map((e) => (
                <tr key={e.entry_id} className="border-t border-slate-800 text-slate-400">
                  <td className="py-1 font-mono text-slate-300">{e.entry_id}</td>
                  <td className="py-1 font-mono">{e.template_id}</td>
                  <td className="py-1">{e.dwell_s ?? group.default_dwell_s ?? 2}</td>
                  <td className="py-1">{e.tactical_mode_hint ?? "—"}</td>
                </tr>
              ))}
            </tbody>
          </table>
          {compiled && (
            <pre className="mb-2 max-h-40 overflow-auto rounded border border-slate-800 bg-slate-950 p-2 text-[10px] text-slate-300">
              {batchSpecToYaml(compiled)}
            </pre>
          )}
          <pre className="overflow-x-auto rounded border border-slate-800 bg-slate-950 p-2 text-[10px] text-emerald-200/90">
            {cliCommand}
          </pre>
        </>
      )}
    </PanelShell>
  );
}
