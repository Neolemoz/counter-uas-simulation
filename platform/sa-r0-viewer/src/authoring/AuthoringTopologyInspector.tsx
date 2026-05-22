import type { CatalogPack } from "@/replay/catalogSchema";

type Props = {
  pack: CatalogPack | null;
};

export function AuthoringTopologyInspector({ pack }: Props) {
  if (!pack) {
    return <p className="text-xs text-slate-500">Select a scenario pack to inspect topology metadata.</p>;
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-slate-300">{pack.title}</p>
      <p className="font-mono text-slate-500">{pack.pack_path}</p>
      {pack.ingress_archetype && (
        <p>
          Ingress: <span className="text-slate-300">{pack.ingress_archetype}</span>
        </p>
      )}
      {pack.terrain_profile && (
        <p>
          Terrain: <span className="text-slate-300">{pack.terrain_profile}</span>
        </p>
      )}
      {pack.replay_duration_class && (
        <p>
          Duration class: <span className="text-slate-300">{pack.replay_duration_class}</span>
        </p>
      )}
      {(pack.overlay_descriptors ?? []).length > 0 && (
        <ul className="list-inside list-disc text-slate-400">
          {pack.overlay_descriptors!.map((d) => (
            <li key={d.kind}>
              {d.kind}
              {d.count != null ? ` ×${d.count}` : ""}
            </li>
          ))}
        </ul>
      )}
      <p className="text-[10px] text-slate-600">
        Read-only inspector — edit topology JSON under fixtures/scenarios/ via CLI workflow.
      </p>
    </div>
  );
}
