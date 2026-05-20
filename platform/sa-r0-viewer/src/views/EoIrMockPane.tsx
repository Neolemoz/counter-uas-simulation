import { useClockStore } from "@/replay/clockStore";
import { getBracketingSamples, interpolatePosition } from "@/replay/trackPlayback";

export function EoIrMockPane() {
  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  if (!bundle) return null;

  const threat = bundle.tracks.find((t) => t.role === "threat");
  const head = threat ? interpolatePosition(threat.samples, currentT) : null;
  const bracket = threat ? getBracketingSamples(threat.samples, currentT) : { kind: "empty" as const };

  return (
    <section className="flex h-full flex-col rounded border border-slate-700 bg-slate-950 p-2">
      <h2 className="text-xs font-semibold uppercase text-slate-500">EO/IR (mock)</h2>
      <p className="mb-1 text-[10px] text-slate-600">Static FOV frame — not live video.</p>
      <div className="relative flex flex-1 items-center justify-center rounded bg-black/60">
        <div className="absolute inset-4 border border-dashed border-slate-600" />
        {head ? (
          <div className="text-center">
            <p className="text-xs text-slate-400">
              Threat @ replay t={currentT}
              {head.interpolated ? " (interpolated)" : ""}
            </p>
            <p className="font-mono text-[10px] text-slate-500">
              ({head.x_m.toFixed(0)}, {head.y_m.toFixed(0)}, {head.z_m.toFixed(0)}) m
            </p>
            {bracket.kind === "between" && (
              <p className="mt-1 text-[10px] text-slate-600">
                Between log samples t={bracket.prev.t}–{bracket.next.t}
              </p>
            )}
          </div>
        ) : (
          <p className="text-xs text-slate-600">No threat sample at current replay time</p>
        )}
      </div>
    </section>
  );
}
