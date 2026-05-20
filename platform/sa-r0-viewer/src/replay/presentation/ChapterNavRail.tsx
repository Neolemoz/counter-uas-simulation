import { useClockStore } from "../clockStore";
import { usePresentationStore } from "./presentationStore";

export function ChapterNavRail() {
  const bundle = useClockStore((s) => s.bundle);
  const chapters = bundle?.presentation?.chapters ?? [];
  const currentChapterIndex = usePresentationStore((s) => s.currentChapterIndex);
  const setChapterIndex = usePresentationStore((s) => s.setChapterIndex);
  const chapter = chapters[currentChapterIndex];

  if (!chapters.length) return null;

  return (
    <section className="rounded border border-violet-900/30 bg-slate-900/80 p-3">
      <div className="mb-2 flex flex-wrap gap-1">
        {chapters.map((ch, i) => (
          <button
            key={ch.chapter_id}
            type="button"
            className={`rounded px-2 py-1 text-xs ${
              i === currentChapterIndex
                ? "bg-violet-800 text-violet-100"
                : "bg-slate-800 text-slate-400 hover:bg-slate-700"
            }`}
            onClick={() => setChapterIndex(i)}
          >
            {ch.title}
          </button>
        ))}
      </div>
      {chapter && (
        <p className="text-sm text-violet-100/90 transition-opacity duration-300">{chapter.summary}</p>
      )}
    </section>
  );
}
