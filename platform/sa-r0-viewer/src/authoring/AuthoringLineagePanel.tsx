import { useEffect, useState } from "react";
import type { CatalogPack } from "@/replay/catalogSchema";
import { loadAuthoringAncestorChain } from "./loadAuthoring";
import type { ScenarioAuthoringManifest } from "./authoringSchema";
import { loadAuthoringManifest } from "./loadAuthoring";

type Props = {
  pack: CatalogPack | null;
};

export function AuthoringLineagePanel({ pack }: Props) {
  const [manifest, setManifest] = useState<ScenarioAuthoringManifest | null>(null);
  const [chain, setChain] = useState<
    { pack_id: string; parent_pack_id: string | null; promotion_status: string }[]
  >([]);

  useEffect(() => {
    if (!pack?.pack_id) {
      setManifest(null);
      setChain([]);
      return;
    }
    loadAuthoringManifest(pack.pack_id)
      .then(setManifest)
      .catch(() => setManifest(null));
    loadAuthoringAncestorChain(pack.pack_id)
      .then(setChain)
      .catch(() => setChain([]));
  }, [pack?.pack_id]);

  if (!pack) {
    return <p className="text-xs text-slate-500">Select a scenario pack.</p>;
  }

  const baseline = pack.baseline_pack_id ?? manifest?.parent_pack_id;
  const depth = chain.length;

  return (
    <div className="space-y-2 text-xs">
      <p className="text-slate-500">Topology derivation (explanatory only)</p>
      {depth > 0 && (
        <p className="text-[10px] text-slate-600">Lineage depth: {depth} hop{depth === 1 ? "" : "s"}</p>
      )}
      {chain.length > 0 ? (
        <ol className="space-y-1 border-l border-slate-700/60 pl-3 font-mono text-slate-300">
          {chain.map((hop) => (
            <li key={hop.pack_id} className="relative">
              <span className="absolute -left-[7px] top-1.5 h-2 w-2 rounded-full bg-slate-600" />
              <span className={hop.pack_id === pack.pack_id ? "text-slate-100" : "text-teal-300/90"}>
                {hop.pack_id}
              </span>
              <span className="block text-[10px] text-slate-600 font-sans">
                {hop.promotion_status}
                {hop.parent_pack_id ? ` · parent ${hop.parent_pack_id}` : ""}
              </span>
            </li>
          ))}
        </ol>
      ) : (
        <ul className="space-y-1 font-mono text-slate-300">
          {baseline && baseline !== pack.pack_id && (
            <li>
              baseline → <span className="text-teal-300/90">{baseline}</span>
            </li>
          )}
          {manifest?.parent_pack_id && manifest.parent_pack_id !== pack.pack_id && (
            <li>
              parent → <span className="text-teal-300/90">{manifest.parent_pack_id}</span>
            </li>
          )}
          <li className="text-slate-200">pack → {pack.pack_id}</li>
        </ul>
      )}
      {!manifest && !baseline && chain.length === 0 && (
        <p className="text-slate-500">Library pack — no derivation chain recorded.</p>
      )}
      <p className="text-[10px] text-slate-600">
        Linkage (compare) and corpus lineage are separate planes — see workflow lineage panel.
      </p>
    </div>
  );
}
