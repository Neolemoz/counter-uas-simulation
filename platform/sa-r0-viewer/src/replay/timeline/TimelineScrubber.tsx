import { useClockStore } from "../clockStore";
import type { PresentationChapter } from "../presentation/presentationStore";
import { sandboxButtons, sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  compressed?: boolean;
  chapter?: PresentationChapter | null;
  presentationMode?: boolean;
};

export function TimelineScrubber({
  compressed = false,
  chapter = null,
  presentationMode = false,
}: Props) {
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
    <section className={`${sandboxSurfaces.panel} p-3`}>
      {presentationMode && compressed && chapter && (
        <p className={`mb-2 ${sandboxTypography.caption} text-violet-200/70`}>
          Chapter window: t {chapter.t_start}–{chapter.t_end}
        </p>
      )}
      <div className="mb-2 flex items-center gap-2">
        <button
          type="button"
          className={presentationMode ? sandboxButtons.primary : sandboxButtons.subtle}
          onClick={() => setPlaying(!playing)}
        >
          {playing ? "Pause" : "Play"}
        </button>
        <span className="text-sm tabular-nums text-slate-400">
          t = {currentT} ({bundle.clock.domain})
        </span>
      </div>
      <input
        type="range"
        min={start}
        max={Math.max(end, start + 1)}
        step={bundle.clock.duration.step}
        value={currentT}
        className="w-full accent-violet-600"
        onChange={(e) => {
          setPlaying(false);
          setCurrentT(Number(e.target.value));
        }}
      />
      <div className="mt-2 flex max-h-20 flex-wrap gap-1 overflow-y-auto">
        {bundle.clock.markers.map((m) => (
          <button
            key={`${m.t}-${m.event_id ?? m.label}`}
            type="button"
            title={m.label}
            className={`rounded px-1.5 py-0.5 sandbox-caption ${
              m.t === currentT
                ? "bg-violet-800/50 text-violet-100"
                : "bg-slate-800/80 text-slate-500 hover:text-slate-300"
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
