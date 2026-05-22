import { useEffect, useState } from "react";
import { loadAuthoringManifest } from "./loadAuthoring";
import type { ScenarioAuthoringManifest } from "./authoringSchema";
import { LADDER_STATUSES, RETIRED_STATUSES } from "./authoringSchema";

const STATUS_LABEL: Record<string, string> = {
  draft: "Draft — edit pack; run CLI lint",
  linted: "Linted — run CLI validate",
  validated: "Validated — CLI may promote",
  promoted: "Promoted — catalog-eligible (CLI sync)",
  orchestration_ready: "Orchestration ready — handoff refs only",
  deprecated: "Deprecated — superseded; read-only",
  archived: "Archived — terminal retired state",
};

const LADDER_RANK: Record<string, number> = Object.fromEntries(
  LADDER_STATUSES.map((s, i) => [s, i]),
);

type Props = {
  packId: string | null;
};

export function AuthoringPromotionPanel({ packId }: Props) {
  const [manifest, setManifest] = useState<ScenarioAuthoringManifest | null>(null);

  useEffect(() => {
    if (!packId) {
      setManifest(null);
      return;
    }
    loadAuthoringManifest(packId)
      .then(setManifest)
      .catch(() => setManifest(null));
  }, [packId]);

  if (!packId) {
    return <p className="text-xs text-slate-500">Select a scenario pack.</p>;
  }

  if (!manifest) {
    return (
      <p className="text-xs text-slate-500">
        No authoring manifest mirror for <span className="font-mono">{packId}</span>. Maintainer:
        run <span className="font-mono">promote_scenario_pack.py --init</span> (CLI only).
      </p>
    );
  }

  const status = manifest.promotion_status;
  const isRetired = RETIRED_STATUSES.includes(status as (typeof RETIRED_STATUSES)[number]);
  const currentRank = LADDER_RANK[status] ?? -1;

  return (
    <div className="space-y-2 text-xs">
      <p className="font-medium text-teal-200/90">AUTHORING MIRROR — CLI promotes; not live state</p>
      {isRetired && (
        <p className="rounded border border-amber-700/40 bg-amber-950/30 px-2 py-1 text-amber-200/90">
          Retired lifecycle — read-only semantics
        </p>
      )}
      {manifest.governance?.notice && (
        <p className="text-slate-500">{manifest.governance.notice}</p>
      )}
      {(manifest.governance?.anti_claims ?? []).length > 0 && (
        <ul className="list-inside list-disc text-[10px] text-slate-600">
          {manifest.governance!.anti_claims!.map((c) => (
            <li key={c}>{c}</li>
          ))}
        </ul>
      )}
      <p>
        Status: <span className="font-mono text-slate-200">{status}</span>
      </p>
      <p className="text-slate-400">{STATUS_LABEL[status] ?? status}</p>
      {!isRetired && (
        <ul className="space-y-0.5 text-[10px] text-slate-600">
          {LADDER_STATUSES.map((step) => {
            const done = LADDER_RANK[step] <= currentRank;
            return (
              <li key={step} className={done ? "text-emerald-500/80" : ""}>
                {done ? "✓" : "○"} {step}
              </li>
            );
          })}
        </ul>
      )}
      {manifest.topology_variant_class && (
        <p>
          Variant class:{" "}
          <span className="font-mono text-slate-300">{manifest.topology_variant_class}</span>
        </p>
      )}
      {manifest.validation_pack_fingerprint && (
        <p className="text-[10px] text-slate-600 break-all">
          Fingerprint: {manifest.validation_pack_fingerprint}
        </p>
      )}
      {manifest.updated_at && (
        <p className="text-[10px] text-slate-600">Updated: {manifest.updated_at}</p>
      )}
      {(manifest.promotion_lineage ?? []).length > 0 && (
        <div>
          <p className="text-slate-500 mb-1">Promotion history</p>
          <ul className="space-y-1 text-slate-400">
            {manifest.promotion_lineage!.map((ev) => (
              <li key={ev.event_id} className="border-l border-slate-700 pl-2">
                <span className="font-mono text-slate-300">
                  {ev.from_status} → {ev.to_status}
                </span>
                {ev.recorded_at && (
                  <span className="block text-[10px] text-slate-600">{ev.recorded_at}</span>
                )}
                {ev.actor && <span className="block text-[10px]">{ev.actor}</span>}
                {ev.notes && <span className="block italic">{ev.notes}</span>}
              </li>
            ))}
          </ul>
        </div>
      )}
      {manifest.authoring_notes && (
        <p className="text-slate-400 italic">{manifest.authoring_notes}</p>
      )}
    </div>
  );
}
