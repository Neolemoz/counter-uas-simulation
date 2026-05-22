import { useEffect, useMemo, useState } from "react";
import { loadOrchestrationIntegrityReport } from "./loadOrchestrationOps";
import type { OrchestrationIntegrityReport } from "./orchestrationOpsSchema";

type Props = {
  manifestId: string | null;
  packId: string | null;
};

export function OrchestrationIntegrityPanel({ manifestId, packId }: Props) {
  const [report, setReport] = useState<OrchestrationIntegrityReport | null>(null);

  useEffect(() => {
    loadOrchestrationIntegrityReport()
      .then(setReport)
      .catch(() => setReport(null));
  }, []);

  const manifestResult = useMemo(() => {
    if (!manifestId || !report?.per_manifest) return null;
    return report.per_manifest[manifestId] ?? null;
  }, [manifestId, report]);

  const packResult = useMemo(() => {
    if (!packId || !report?.per_pack) return null;
    return report.per_pack[packId] ?? null;
  }, [packId, report]);

  if (!report) {
    return (
      <p className="text-xs text-slate-500">
        No integrity report. Run <span className="font-mono">sync_orchestration_mirrors.py</span>.
      </p>
    );
  }

  const errors = [
    ...(manifestResult?.errors ?? []),
    ...(packResult?.errors ?? []),
  ];
  const warnings = [
    ...(manifestResult?.warnings ?? []),
    ...(packResult?.warnings ?? []),
  ];

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-violet-200/90">
        {report.governance_banner ?? "ORCHESTRATION INTEGRITY — explanatory only"}
      </p>
      <p className={report.ok ? "text-emerald-400/90" : "text-amber-300/80"}>
        Corpus: {report.manifest_count ?? "?"}/{report.pack_count ?? "?"} manifests ·{" "}
        {report.ok ? "pass" : "issues"}
      </p>
      {errors.length === 0 && warnings.length === 0 && (
        <p className="text-emerald-400/80">No scoped issues in last audit.</p>
      )}
      {errors.map((e) => (
        <p key={e} className="text-red-300/90">
          {e}
        </p>
      ))}
      {warnings.map((w) => (
        <p key={w} className="text-amber-200/80">
          {w}
        </p>
      ))}
    </div>
  );
}
