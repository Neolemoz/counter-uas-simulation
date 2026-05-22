import type { CatalogPack } from "./catalogSchema";
import { CATEGORY_LABELS } from "./catalogSchema";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { navigateToComparePair } from "@/navigation/experimentNavigation";

type Props = {
  entry: CatalogPack;
  hooks?: ExperimentNavHooks;
};

export function ScenarioPreviewCard({ entry, hooks }: Props) {
  const category = entry.category ? (CATEGORY_LABELS[entry.category] ?? entry.category) : null;
  const pairIds = entry.compare_pair_ids ?? [];

  return (
    <div className="rounded border border-slate-700 bg-slate-800/50 p-2 text-xs text-slate-400">
      {category && (
        <p>
          <span className="font-medium text-slate-500">Category: </span>
          {category}
        </p>
      )}
      {entry.ingress_archetype && (
        <p>
          <span className="font-medium text-slate-500">Ingress: </span>
          {entry.ingress_archetype}
        </p>
      )}
      {entry.replay_duration_class && (
        <p>
          <span className="font-medium text-slate-500">Duration: </span>
          {entry.replay_duration_class}
        </p>
      )}
      {entry.terrain_profile && (
        <p>
          <span className="font-medium text-slate-500">Terrain: </span>
          {entry.terrain_profile}
        </p>
      )}
      {entry.ambiguity_profile?.level && (
        <p>
          <span className="font-medium text-slate-500">Ambiguity: </span>
          {entry.ambiguity_profile.level}
        </p>
      )}
      {(entry.narrative_focus?.length ?? 0) > 0 && (
        <p className="mt-1 text-slate-500">{entry.narrative_focus!.join(" · ")}</p>
      )}
      {pairIds.length > 0 && hooks && (
        <div className="mt-2 flex flex-wrap gap-1">
          {pairIds.map((pairId) => (
            <button
              key={pairId}
              type="button"
              className="rounded bg-amber-900/40 px-1.5 py-0.5 text-[10px] text-amber-100 hover:bg-amber-900/60"
              onClick={() => void navigateToComparePair(pairId, hooks)}
            >
              Compare: {pairId}
            </button>
          ))}
        </div>
      )}
    </div>
  );
}
