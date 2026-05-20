import { useClockStore } from "@/replay/clockStore";
import { interpolatePosition } from "@/replay/trackPlayback";

export function RadarMockPane() {
  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  if (!bundle) return null;

  const points: { x: number; y: number; role: string; interpolated: boolean }[] = [];
  for (const tr of bundle.tracks) {
    const pos = interpolatePosition(tr.samples, currentT);
    if (pos) {
      points.push({
        x: pos.y_m,
        y: -pos.x_m,
        role: tr.role,
        interpolated: pos.interpolated,
      });
    }
  }

  return (
    <section className="flex h-full flex-col rounded border border-slate-700 bg-slate-950 p-2">
      <h2 className="text-xs font-semibold uppercase text-slate-500">Radar view (mock)</h2>
      <p className="mb-1 text-[10px] text-slate-600">Replay-derived PPI — not live radar.</p>
      <svg viewBox="-120 -120 240 240" className="mx-auto flex-1 max-h-48 w-full">
        <circle cx="0" cy="0" r="100" fill="none" stroke="#334155" strokeWidth="1" />
        <circle cx="0" cy="0" r="60" fill="none" stroke="#334155" strokeWidth="0.5" />
        <line x1="0" y1="-100" x2="0" y2="100" stroke="#334155" strokeWidth="0.5" />
        <line x1="-100" y1="0" x2="100" y2="0" stroke="#334155" strokeWidth="0.5" />
        {points.map((p, i) => {
          const scale = 0.04;
          const cx = Math.max(-95, Math.min(95, p.x * scale));
          const cy = Math.max(-95, Math.min(95, p.y * scale));
          return (
            <circle
              key={`${p.role}-${i}`}
              cx={cx}
              cy={cy}
              r={p.role === "threat" ? 5 : 4}
              fill={p.role === "threat" ? "#f97316" : "#22d3ee"}
              opacity={p.interpolated ? 0.85 : 1}
            />
          );
        })}
      </svg>
      <p className="text-center text-[10px] text-slate-600">t={currentT} — sparse sample interpolation</p>
    </section>
  );
}
