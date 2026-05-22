import { useState } from "react";
import type { FilmstripSlotState } from "../cohortFilmstripStore";
import { CesiumReplayMap, type MapReplayState } from "@/cesium/CesiumReplayMap";

type Props = {
  slot: FilmstripSlotState;
  label: string;
};

export function FilmstripSlotMapPane({ slot, label }: Props) {
  const [interceptorMissing, setInterceptorMissing] = useState(false);
  const bundle = slot.bundle;
  if (!bundle) {
    return (
      <section className="flex min-h-[200px] flex-1 items-center justify-center rounded border border-dashed border-slate-700 text-slate-500">
        {label}
      </section>
    );
  }

  const replayState: MapReplayState = {
    currentT: slot.currentT,
    layers: slot.layers,
    highlightedTrackIds: slot.highlightedTrackIds,
    selectedEventId: slot.selectedEventId,
    losScope: slot.losScope,
    fitReplayNonce: slot.fitReplayNonce,
  };

  return (
    <section className="flex min-h-[200px] min-w-0 flex-1 flex-col overflow-hidden rounded border border-slate-700">
      <div className="border-b border-slate-700 bg-slate-900 px-2 py-1 text-xs font-semibold uppercase text-slate-500">
        {label}
      </div>
      {interceptorMissing && (
        <p className="border-b border-amber-900/50 bg-amber-950/40 px-2 py-0.5 text-[10px] text-amber-200/90">
          No interceptor samples
        </p>
      )}
      <div className="relative min-h-0 flex-1">
        <CesiumReplayMap
          bundle={bundle}
          className="absolute inset-0 h-full w-full"
          replayState={replayState}
          onInterceptorSamplesMissing={setInterceptorMissing}
        />
      </div>
    </section>
  );
}
