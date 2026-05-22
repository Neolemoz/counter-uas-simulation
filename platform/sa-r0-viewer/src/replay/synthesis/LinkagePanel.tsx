import { useEffect, useState } from "react";
import { useSweepStore } from "../useSweepStore";
import { linkageEdgesForSweep, loadReplayLinkageIndex, relatedSweepIds } from "./loadSynthesis";
import type { ReplayLinkageIndex } from "./synthesisSchema";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { navigateSweepById } from "@/navigation/experimentNavigation";

type Props = {
  hooks?: ExperimentNavHooks;
};

export function LinkagePanel({ hooks }: Props) {
  const sweepId = useSweepStore((s) => s.sweep?.sweep_id);
  const [linkage, setLinkage] = useState<ReplayLinkageIndex | null>(null);

  useEffect(() => {
    loadReplayLinkageIndex()
      .then(setLinkage)
      .catch(() => setLinkage(null));
  }, []);

  if (!linkage || !sweepId) return null;

  const edges = linkageEdgesForSweep(linkage, sweepId);
  const related = relatedSweepIds(linkage, sweepId);
  if (edges.length === 0 && related.length === 0) return null;

  return (
    <section className="rounded border border-indigo-900/40 bg-indigo-950/20 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-indigo-100">Related replay families</h2>
      <p className="mb-2 text-xs text-slate-400">
        Rule-based linkage — descriptive similarity only, not causal inference.
      </p>
      <div className="mb-2 flex flex-wrap gap-1">
        {related.map((id) =>
          hooks ? (
            <button
              key={id}
              type="button"
              className="rounded bg-slate-800 px-2 py-0.5 text-[10px] text-indigo-200 hover:bg-slate-700"
              onClick={() => void navigateSweepById(id, hooks)}
            >
              {id}
            </button>
          ) : (
            <a
              key={id}
              href={`?sweep=${id}`}
              className="rounded bg-slate-800 px-2 py-0.5 text-[10px] text-indigo-200 hover:bg-slate-700"
            >
              {id}
            </a>
          ),
        )}
      </div>
      <ul className="list-inside list-disc space-y-1 text-xs text-slate-300">
        {edges.slice(0, 6).map((e) => (
          <li key={e.edge_id}>
            <span className="text-slate-500">[{e.link_kind}]</span> {e.copy}
          </li>
        ))}
      </ul>
    </section>
  );
}
