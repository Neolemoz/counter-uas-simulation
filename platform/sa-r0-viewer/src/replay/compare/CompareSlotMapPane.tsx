import { useState } from "react";
import type { Cartesian3 } from "cesium";
import type { CompareSlotId } from "../compareStore";
import { useCompareStore } from "../compareStore";
import { CesiumReplayMap, type MapReplayState } from "@/cesium/CesiumReplayMap";
import type { TopologyDiffHighlight } from "../topologyDiff";

type Props = {
  slot: CompareSlotId;
  label: string;
  diffHighlight?: TopologyDiffHighlight;
  isCameraLeader?: boolean;
  followCamera?: { position: Cartesian3; direction: Cartesian3; up: Cartesian3 } | null;
  onCameraMatrix?: (state: {
    position: Cartesian3;
    direction: Cartesian3;
    up: Cartesian3;
  }) => void;
};

export function CompareSlotMapPane({
  slot,
  label,
  diffHighlight,
  isCameraLeader,
  followCamera,
  onCameraMatrix,
}: Props) {
  const slotState = useCompareStore((s) => (slot === "A" ? s.slotA : s.slotB));
  const emphasizeDelta = useCompareStore((s) => s.emphasizeDelta);
  const cameraLocked = useCompareStore((s) => s.cameraLocked);
  const requestSlotFitReplay = useCompareStore((s) => s.requestSlotFitReplay);
  const [interceptorMissing, setInterceptorMissing] = useState(false);

  const bundle = slotState.bundle;
  if (!bundle) {
    return (
      <section className="flex min-h-[280px] flex-1 items-center justify-center rounded border border-dashed border-slate-700 text-slate-500">
        No bundle — {label}
      </section>
    );
  }

  const replayState: MapReplayState = {
    currentT: slotState.currentT,
    layers: slotState.layers,
    highlightedTrackIds: slotState.highlightedTrackIds,
    selectedEventId: slotState.selectedEventId,
    losScope: slotState.losScope,
    fitReplayNonce: slotState.fitReplayNonce,
  };

  return (
    <section className="flex min-h-[280px] flex-1 flex-col overflow-hidden rounded border border-slate-700">
      <div className="flex items-center justify-between border-b border-slate-700 bg-slate-900 px-2 py-1">
        <h2 className="text-xs font-semibold uppercase text-slate-500">{label}</h2>
        <button
          type="button"
          className="rounded bg-slate-800 px-2 py-0.5 text-xs text-slate-300 hover:bg-slate-700"
          onClick={() => requestSlotFitReplay(slot)}
        >
          Fit
        </button>
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
          diffHighlight={diffHighlight}
          emphasizeDelta={emphasizeDelta}
          isCameraLeader={isCameraLeader}
          cameraLocked={cameraLocked}
          onCameraMatrix={onCameraMatrix}
          followCamera={followCamera}
          onInterceptorSamplesMissing={setInterceptorMissing}
        />
        <div className="pointer-events-none absolute bottom-1 left-1 rounded bg-slate-950/80 px-1.5 py-0.5 text-[9px] text-slate-500">
          {bundle.scenario.title}
        </div>
      </div>
    </section>
  );
}
