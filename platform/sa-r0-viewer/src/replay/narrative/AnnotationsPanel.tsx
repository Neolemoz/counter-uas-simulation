import { useMemo, useState } from "react";
import { useClockStore } from "../clockStore";

const KIND_LABELS: Record<string, string> = {
  los_blockage: "LOS",
  visibility_degraded: "Visibility",
  tti_explanation: "TTI",
};

function rankOf(ann: Record<string, unknown>): number {
  const r = ann.narrative_rank;
  return typeof r === "number" ? r : 99;
}

export function AnnotationsPanel({ spotlightIds = [] }: { spotlightIds?: string[] }) {
  const bundle = useClockStore((s) => s.bundle);
  const currentT = useClockStore((s) => s.currentT);
  const selectedEventId = useClockStore((s) => s.selectedEventId);
  const setSelectedEventId = useClockStore((s) => s.setSelectedEventId);
  const setCurrentT = useClockStore((s) => s.setCurrentT);
  const [expanded, setExpanded] = useState(false);

  const { groups, flatCount } = useMemo(() => {
    if (!bundle) return { groups: [], flatCount: 0 };
    const annotations = [...(bundle.narrative.annotations ?? [])].sort(
      (a, b) => rankOf(a) - rankOf(b),
    );
    const byEvent = new Map<string, Record<string, unknown>[]>();
    const ungrouped: Record<string, unknown>[] = [];
    for (const ann of annotations) {
      const linked = (ann.linked_event_ids as string[] | undefined) ?? [];
      const key = linked[0];
      if (key && (ann.narrative_rank === 2 || linked.length > 0)) {
        const bucket = byEvent.get(key) ?? [];
        bucket.push(ann);
        byEvent.set(key, bucket);
      } else {
        ungrouped.push(ann);
      }
    }
    const groups: { key: string; items: Record<string, unknown>[] }[] = [];
    for (const [key, items] of byEvent) {
      if (items.length > 1) {
        groups.push({ key, items });
      } else {
        ungrouped.push(items[0]!);
      }
    }
    return { groups, flatCount: ungrouped.length + groups.reduce((n, g) => n + g.items.length, 0) };
  }, [bundle]);

  if (!bundle) return null;

  const bookmarks = bundle.narrative.bookmarks ?? [];
  const collapseLowRank = flatCount > 5 && !expanded;

  const isActive = (ann: Record<string, unknown>) => {
    const linked = (ann.linked_event_ids as string[] | undefined) ?? [];
    if (selectedEventId && linked.includes(selectedEventId)) return true;
    for (const eid of linked) {
      const ev = bundle.narrative.events.find((e) => e.event_id === eid);
      if (ev && typeof ev.line_index === "number" && ev.line_index <= currentT) return true;
    }
    return false;
  };

  const jumpToAnnotation = (ann: Record<string, unknown>) => {
    const linked = (ann.linked_event_ids as string[] | undefined) ?? [];
    if (!linked.length) return;
    const eventId = linked[0];
    const ev = bundle.narrative.events.find((e) => e.event_id === eventId);
    setSelectedEventId(eventId);
    if (ev && typeof ev.line_index === "number") {
      setCurrentT(ev.line_index);
    }
  };

  const renderAnn = (a: Record<string, unknown>, dimmed = false) => {
    const kind = String(a.kind ?? "");
    const chip = KIND_LABELS[kind];
    const active = isActive(a);
    const rank = a.narrative_rank;
    const annId = String(a.annotation_id ?? "");
    const spotlight = spotlightIds.length > 0;
    const inSpotlight = !spotlight || spotlightIds.includes(annId);
    return (
      <li key={annId}>
        <button
          type="button"
          className={`w-full rounded p-2 text-left transition-opacity duration-300 hover:bg-slate-700/80 ${
            active ? "border border-amber-700/60 bg-amber-950/30" : "bg-slate-800/80"
          } ${dimmed || !inSpotlight ? "opacity-40" : ""}`}
          onClick={() => jumpToAnnotation(a)}
          disabled={!((a.linked_event_ids as string[] | undefined)?.length)}
        >
          <div className="mb-1 flex items-center gap-2">
            <p className="font-medium text-amber-200">{String(a.title)}</p>
            {typeof rank === "number" && rank <= 2 && (
              <span className="rounded bg-slate-700 px-1 text-[10px] text-slate-400">
                emphasis {rank}
              </span>
            )}
            {chip && (
              <span className="rounded bg-slate-700 px-1.5 py-0.5 text-[10px] uppercase text-slate-400">
                {chip}
              </span>
            )}
          </div>
          <p className="text-slate-400">{String(a.body)}</p>
        </button>
      </li>
    );
  };

  const sortedFlat = [...(bundle.narrative.annotations ?? [])]
    .filter((a) => {
      const linked = (a.linked_event_ids as string[] | undefined) ?? [];
      const key = linked[0];
      const group = groups.find((g) => g.key === key);
      return !group;
    })
    .sort((a, b) => rankOf(a) - rankOf(b));

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Replay annotations</h2>
      <ul className="mb-3 space-y-2">
        {groups.map((g) => (
          <li key={g.key} className="rounded border border-slate-700/80 p-1">
            <p className="mb-1 px-1 text-[10px] font-semibold uppercase text-slate-500">
              Ambiguity window
            </p>
            <ul className="space-y-1">{g.items.map((a) => renderAnn(a))}</ul>
          </li>
        ))}
        {sortedFlat.map((a, i) => {
          const dim = collapseLowRank && rankOf(a) >= 3 && i >= 3;
          return renderAnn(a, dim);
        })}
      </ul>
      {flatCount > 5 && (
        <button
          type="button"
          className="mb-2 text-xs text-slate-500 hover:text-slate-300"
          onClick={() => setExpanded((v) => !v)}
        >
          {expanded ? "Collapse lower emphasis" : "Show all annotations"}
        </button>
      )}
      <h3 className="mb-1 text-xs font-semibold uppercase text-slate-500">Bookmarks</h3>
      <ul className="flex flex-wrap gap-1">
        {bookmarks.map((b) => (
          <li key={String(b.bookmark_id)}>
            <button
              type="button"
              className="rounded bg-slate-700 px-2 py-0.5 text-xs hover:bg-slate-600"
              onClick={() => useClockStore.getState().setCurrentT(Number(b.t))}
            >
              {String(b.label)} @ {String(b.t)}
            </button>
          </li>
        ))}
      </ul>
    </section>
  );
}
