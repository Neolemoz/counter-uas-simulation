import type { CatalogPack } from "./catalogSchema";
import { CATEGORY_LABELS } from "./catalogSchema";

type Props = {
  entry: CatalogPack;
};

export function ScenarioPreviewCard({ entry }: Props) {
  const category = entry.category ? (CATEGORY_LABELS[entry.category] ?? entry.category) : null;
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
    </div>
  );
}
