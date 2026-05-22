import { useCallback, useState } from "react";
import type { Viewer } from "cesium";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { CesiumReplayMap } from "@/cesium/CesiumReplayMap";
import { LOS_LEGEND } from "@/cesium/losSegmentLayer";
import { useClockStore } from "@/replay/clockStore";
import { sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  bundle: ReplaySaBundle;
  presentationDimming?: boolean;
  onViewerReady?: (viewer: Viewer | null) => void;
};

export function StrategicMapPane({ bundle, presentationDimming, onViewerReady }: Props) {
  const requestFitReplay = useClockStore((s) => s.requestFitReplay);
  const [interceptorMissing, setInterceptorMissing] = useState(false);

  const handleViewer = useCallback(
    (viewer: Viewer | null) => {
      onViewerReady?.(viewer);
    },
    [onViewerReady],
  );

  return (
    <section
      className={
        presentationDimming
          ? "flex h-full min-h-[420px] flex-col overflow-hidden rounded-lg border border-slate-700/50 bg-slate-950"
          : "flex h-full min-h-[360px] flex-col overflow-hidden rounded-lg border border-slate-700"
      }
    >
      {presentationDimming ? (
        <p className={`border-b border-slate-800/60 px-3 py-1.5 ${sandboxTypography.caption}`}>
          Replay map — explanatory geometry only, not live sensor truth.
        </p>
      ) : (
        <div className="flex items-center justify-between border-b border-slate-700 bg-slate-900/80 px-2 py-1">
          <h2 className={sandboxTypography.sectionLabel}>Strategic replay map</h2>
          <div className="flex gap-1">
            <button
              type="button"
              className="rounded-md border border-slate-600/80 bg-slate-800/80 px-2 py-0.5 text-xs text-slate-300 hover:bg-slate-700/80"
              onClick={() => requestFitReplay()}
            >
              Fit replay
            </button>
          </div>
        </div>
      )}
      {interceptorMissing && (
        <p className="border-b border-amber-900/40 bg-amber-950/30 px-2 py-1 text-xs text-amber-200/80">
          No interceptor trajectory samples in bundle.
        </p>
      )}
      <div className="relative min-h-0 flex-1">
        <CesiumReplayMap
          bundle={bundle}
          className="absolute inset-0 h-full w-full"
          onInterceptorSamplesMissing={setInterceptorMissing}
          visualProfile={presentationDimming ? "publication" : "default"}
          onViewerReady={handleViewer}
        />
      </div>
      {!presentationDimming && (
        <p className="border-t border-slate-800 px-2 py-1 text-[10px] text-slate-600">
          {LOS_LEGEND.map((e) => e.label).join(" · ")}
        </p>
      )}
    </section>
  );
}
