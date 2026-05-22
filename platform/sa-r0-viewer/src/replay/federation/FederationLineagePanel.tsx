import { useEffect, useState } from "react";
import { loadFederationLineageGraph } from "./loadFederationArtifacts";
import type { ReplayFederationLineageGraph } from "./federationSchema";
import { useFederationStore } from "./useFederationStore";

export function FederationLineagePanel() {
  const [graph, setGraph] = useState<ReplayFederationLineageGraph | null>(null);
  const highlighted = useFederationStore((s) => s.highlightedLineageRef);

  useEffect(() => {
    loadFederationLineageGraph().then(setGraph).catch(() => setGraph(null));
  }, []);

  if (!graph?.edges?.length) {
    return (
      <p className="text-xs text-slate-500">No federation lineage edges in mirror.</p>
    );
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="text-slate-400">Cross-corpus structural edges (not semantic linkage)</p>
      <ul className="space-y-1">
        {graph.edges.map((e) => (
          <li
            key={e.edge_id}
            className={`rounded border px-2 py-1 font-mono text-[10px] ${
              highlighted === e.edge_id
                ? "border-violet-600/50 bg-violet-950/30"
                : "border-slate-800 text-slate-500"
            }`}
          >
            <span className="text-violet-300/80">{e.ref_kind}</span>
            <span className="block text-slate-400">
              {e.from_corpus_group_id} → {e.to_corpus_group_id}
            </span>
            {e.evidence?.note && (
              <span className="block text-slate-600">{e.evidence.note}</span>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
}
