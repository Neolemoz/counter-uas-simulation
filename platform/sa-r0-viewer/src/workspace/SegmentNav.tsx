import { cn } from "@/lib/utils";
import { segmentBanner } from "./segmentBanners";
import { SEGMENT_LABELS, SEGMENT_ORDER, type WorkspaceSegment } from "./types";

type Props = {
  active: WorkspaceSegment;
  onSelect: (segment: WorkspaceSegment) => void;
  locked?: WorkspaceSegment | null;
  authoringMirror?: boolean;
};

export function SegmentNav({ active, onSelect, locked, authoringMirror = false }: Props) {
  return (
    <nav
      className="sandbox-no-print border-b border-slate-800/80 bg-slate-950/90 px-3 py-2"
      aria-label="Workspace segment"
    >
      <div className="flex flex-wrap items-end gap-0.5">
        {SEGMENT_ORDER.map((seg) => {
          const isActive = active === seg;
          const isLocked = locked === seg;
          return (
            <button
              key={seg}
              type="button"
              disabled={isLocked}
              className={cn(
                "sandbox-segment-tab",
                isActive && "sandbox-segment-tab-active",
                isLocked && "cursor-default opacity-90",
              )}
              aria-current={isActive ? "page" : undefined}
              onClick={() => onSelect(seg)}
            >
              {SEGMENT_LABELS[seg]}
            </button>
          );
        })}
      </div>
      <p className="mt-2 text-[11px] leading-snug text-slate-500">
        {segmentBanner(active, { authoringMirror })}
      </p>
    </nav>
  );
}
