import { useClockStore } from "../clockStore";
import type { PresentationChapter } from "../presentation/presentationStore";

type Props = {
  compressed?: boolean;
  chapter?: PresentationChapter | null;
};

export function TimelineScrubber({ compressed = false, chapter = null }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  const playing = useClockStore((s) => s.playing);
  const setCurrentT = useClockStore((s) => s.setCurrentT);
  const setPlaying = useClockStore((s) => s.setPlaying);
  const setSelectedEventId = useClockStore((s) => s.setSelectedEventId);

  if (!bundle) return null;
  const full = bundle.clock.duration;
  const start = compressed && chapter ? chapter.t_start : full.start;
  const end = compressed && chapter ? chapter.t_end : full.end;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3">
      <div className="mb-2 flex items-center gap-2">
        <button
          type="button"
          className="rounded bg-slate-700 px-3 py-1 text-sm hover:bg-slate-600"
          onClick={() => setPlaying(!playing)}
        >
          {playing ? "Pause" : "Play"}
        </button>
        <span className="text-sm text-slate-400">
          t = {currentT} ({bundle.clock.domain})
        </span>
      </div>
      <input
        type="range"
        min={start}
        max={Math.max(end, start + 1)}
        step={bundle.clock.duration.step}
        value={currentT}
        className="w-full"
        onChange={(e) => {
          setPlaying(false);
          setCurrentT(Number(e.target.value));
        }}
      />
      <div className="mt-1 flex flex-wrap gap-1">
        {bundle.clock.markers.map((m) => (
          <button
            key={`${m.t}-${m.event_id ?? m.label}`}
            type="button"
            title={m.label}
            className={`rounded px-1.5 py-0.5 text-xs ${
              m.t === currentT ? "bg-amber-700 text-white" : "bg-slate-800 text-slate-400"
            }`}
            onClick={() => {
              setPlaying(false);
              setCurrentT(m.t);
              if (m.event_id) setSelectedEventId(String(m.event_id));
            }}
          >
            {m.category?.slice(0, 3) ?? "·"}@{m.t}
          </button>
        ))}
      </div>
    </section>
  );
}
