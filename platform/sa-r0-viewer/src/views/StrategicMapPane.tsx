import { useState } from "react";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { CesiumReplayMap } from "@/cesium/CesiumReplayMap";
import { LOS_LEGEND } from "@/cesium/losSegmentLayer";
import { useClockStore } from "@/replay/clockStore";

type Props = { bundle: ReplaySaBundle; presentationDimming?: boolean };

export function StrategicMapPane({ bundle, presentationDimming }: Props) {
  const requestFitReplay = useClockStore((s) => s.requestFitReplay);
  const [interceptorMissing, setInterceptorMissing] = useState(false);

  const resetView = () => {
    requestFitReplay();
  };

  return (
    <section className="flex h-full min-h-[360px] flex-col overflow-hidden rounded border border-slate-700">
      {!presentationDimming && (
        <div className="flex items-center justify-between border-b border-slate-700 bg-slate-900 px-2 py-1">
          <h2 className="text-xs font-semibold uppercase text-slate-500">Strategic replay map</h2>
          <div className="flex gap-1">
            <button
              type="button"
              className="rounded bg-slate-800 px-2 py-0.5 text-xs text-slate-300 hover:bg-slate-700"
              onClick={() => requestFitReplay()}
            >
              Fit replay
            </button>
            <button
              type="button"
              className="rounded bg-slate-800 px-2 py-0.5 text-xs text-slate-300 hover:bg-slate-700"
              onClick={resetView}
              title="Same as Fit replay — frames all track samples"
            >
              Reset view
            </button>
          </div>
        </div>
      )}
      {interceptorMissing && (
        <p className="border-b border-amber-900/50 bg-amber-950/40 px-2 py-1 text-xs text-amber-200/90">
          No interceptor trajectory samples in bundle.
        </p>
      )}
      <div className="relative min-h-0 flex-1">
        <CesiumReplayMap
          bundle={bundle}
          className="absolute inset-0 h-full w-full"
          onInterceptorSamplesMissing={setInterceptorMissing}
        />
        <div className="pointer-events-none absolute bottom-2 left-2 rounded bg-slate-950/80 px-2 py-1 text-[10px] text-slate-400">
          <p className="mb-0.5 font-semibold uppercase text-slate-500">LOS (explanatory)</p>
          <ul className="space-y-0.5">
            {LOS_LEGEND.map((row) => (
              <li key={row.status}>{row.label}</li>
            ))}
          </ul>
        </div>
      </div>
    </section>
  );
}
