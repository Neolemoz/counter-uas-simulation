import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { useClockStore } from "@/replay/clockStore";
import { getOnboardPhase, ONBOARD_PHASE_COPY } from "@/replay/onboardPhase";
import { interpolatePosition } from "@/replay/trackPlayback";

const PHASE_VISUAL: Record<string, string> = {
  no_samples: "○",
  pre_launch: "▷",
  en_route: "◆",
  target_acquired: "◎",
  intercept_window: "✦",
};

type Props = {
  bundle?: ReplaySaBundle | null;
  currentT?: number;
};

export function InterceptorCameraMockPane({ bundle: bundleProp, currentT: tProp }: Props = {}) {
  const bundle = bundleProp ?? useClockStore((s) => s.bundle);
  const currentT = tProp ?? useClockStore((s) => s.currentT);
  if (!bundle) return null;

  const phase = getOnboardPhase(bundle, currentT);
  const copy = ONBOARD_PHASE_COPY[phase];
  const int = bundle.tracks.find((t) => t.role === "interceptor");
  const head = int ? interpolatePosition(int.samples, currentT) : null;

  return (
    <section className="flex h-full flex-col rounded border border-slate-700 bg-slate-950 p-2">
      <h2 className="text-xs font-semibold uppercase text-slate-500">Onboard camera (mock)</h2>
      <p className="mb-1 text-[10px] text-amber-700/90">
        Recorded/mock replay frame — not live feed.
      </p>
      <div className="flex flex-1 flex-col items-center justify-center rounded bg-gradient-to-b from-slate-800 to-slate-950 p-3">
        <span className="mb-2 text-3xl text-slate-400">{PHASE_VISUAL[phase]}</span>
        <p className="text-center text-sm font-medium text-slate-200">{copy.title}</p>
        <p className="mt-1 text-center text-xs text-slate-500">{copy.detail}</p>
        {head && phase !== "no_samples" && phase !== "pre_launch" && (
          <p className="mt-2 font-mono text-[10px] text-slate-600">
            replay t={currentT}
            {head.interpolated ? " (interpolated)" : ""} — ({head.x_m.toFixed(0)}, {head.y_m.toFixed(0)}) m
          </p>
        )}
      </div>
    </section>
  );
}
