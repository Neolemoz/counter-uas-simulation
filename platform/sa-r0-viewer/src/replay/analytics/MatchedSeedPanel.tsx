import { useEffect, useState } from "react";
import { useSweepStore } from "../useSweepStore";

type MatchedSeedReport = {
  artifact_type: string;
  summary?: { paired_seed_count?: number };
  paired_seeds?: Array<{ seed?: number }>;
  interpretation_caveats?: string[];
};

export function MatchedSeedPanel() {
  const sweep = useSweepStore((s) => s.sweep);
  const [report, setReport] = useState<MatchedSeedReport | null>(null);

  useEffect(() => {
    if (!sweep || sweep.sweep_kind !== "matched_seed") {
      setReport(null);
      return;
    }
    fetch(`/demo/sweeps/${sweep.sweep_id}/matched_seed_report.json`)
      .then((r) => (r.ok ? r.json() : null))
      .then((d) => setReport(d as MatchedSeedReport | null))
      .catch(() => setReport(null));
  }, [sweep]);

  if (!report) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-slate-200">Matched-seed report (fixture)</h2>
      <p className="text-xs text-slate-400">
        Paired seeds: {report.summary?.paired_seed_count ?? "—"} — derived fixture, not parser authority.
      </p>
      <ul className="mt-2 list-inside list-disc text-[10px] text-slate-500">
        {(report.interpretation_caveats ?? []).map((c) => (
          <li key={c}>{c}</li>
        ))}
      </ul>
    </section>
  );
}
