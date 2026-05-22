import { useClockStore } from "../clockStore";
import { usePresentationStore } from "./presentationStore";
import { sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";
import { cn } from "@/lib/utils";

export function ChapterNavRail() {
  const bundle = useClockStore((s) => s.bundle);
  const chapters = bundle?.presentation?.chapters ?? [];
  const currentChapterIndex = usePresentationStore((s) => s.currentChapterIndex);
  const setChapterIndex = usePresentationStore((s) => s.setChapterIndex);
  const chapter = chapters[currentChapterIndex];

  if (!chapters.length) return null;

  return (
    <section className={cn(sandboxSurfaces.panel, "border-violet-900/30 p-3")}>
      <p className={`mb-2 ${sandboxTypography.sectionLabel} text-violet-300/80`}>Chapters</p>
      <ol className="mb-3 space-y-1">
        {chapters.map((ch, i) => (
          <li key={ch.chapter_id}>
            <button
              type="button"
              className={cn(
                "flex w-full items-start gap-2 rounded-md px-2 py-1.5 text-left text-xs transition-colors",
                i === currentChapterIndex
                  ? "bg-violet-900/35 text-violet-100"
                  : "text-slate-400 hover:bg-slate-800/50 hover:text-slate-200",
              )}
              onClick={() => setChapterIndex(i)}
            >
              <span
                className={cn(
                  "mt-0.5 flex h-5 w-5 shrink-0 items-center justify-center rounded-full text-[10px] font-medium tabular-nums",
                  i === currentChapterIndex
                    ? "bg-violet-700/60 text-violet-50"
                    : "bg-slate-800 text-slate-500",
                )}
              >
                {i + 1}
              </span>
              <span className="leading-snug">{ch.title}</span>
            </button>
          </li>
        ))}
      </ol>
      {chapter && (
        <p className={`transition-opacity duration-300 ${sandboxTypography.chapterSummary}`}>
          {chapter.summary}
        </p>
      )}
    </section>
  );
}
