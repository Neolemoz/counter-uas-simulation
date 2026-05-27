import { PanelShell } from "./GovernanceChrome";
import { TelemetryCognitionStrip } from "./TelemetryCognitionStrip";
import type { ChannelSnapshot } from "@/telemetry/channelIndex";
import { entitiesFromSnapshot } from "@/telemetry/channelIndex";
import { ENTITY_GLYPHS } from "@/world/entityCatalog";
import { renderAsciiGrid } from "@/world/gridCoords";

export function EntityPoseMirrorPanel({
  snapshot,
  hideCognition = false,
}: {
  snapshot: ChannelSnapshot | undefined;
  hideCognition?: boolean;
}) {
  const entities = entitiesFromSnapshot(snapshot);
  const gridLines = renderAsciiGrid(
    entities.map((e) => ({
      entity_type: String(e.entity_type ?? ""),
      pose: e.pose as Record<string, unknown>,
    })),
    ENTITY_GLYPHS,
  );

  return (
    <PanelShell title="Entity pose mirror (read-only)">
      <p className="mb-2 text-xs text-slate-500">
        Explanatory mirror snapshot — use world editor above for registry commands.
      </p>
      <pre className="overflow-x-auto rounded bg-slate-950 p-2 font-mono text-[10px] leading-tight text-emerald-300">
        {gridLines.join("\n")}
      </pre>
      {!hideCognition && <TelemetryCognitionStrip snapshot={snapshot} />}
    </PanelShell>
  );
}
