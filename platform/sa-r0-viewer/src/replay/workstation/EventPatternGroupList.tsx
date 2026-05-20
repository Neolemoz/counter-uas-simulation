import { useSweepStore } from "../useSweepStore";

const TAG_LABELS: Record<string, string> = {
  los_fragmented_replay: "LOS fragmented",
  assignment_instability_replay: "Assignment instability",
  delayed_detection_replay: "Delayed detection",
  corridor_pressure_replay: "Corridor pressure",
  saturation_driven_ambiguity: "Saturation ambiguity",
  topology_sensitive_divergence: "Topology divergence",
};

export function EventPatternGroupList() {
  const sweep = useSweepStore((s) => s.sweep);
  const filterTags = useSweepStore((s) => s.cohortFilterTags);
  const toggleTag = useSweepStore((s) => s.toggleCohortFilterTag);
  if (!sweep) return null;

  const allTags = new Set<string>();
  for (const m of sweep.members) {
    for (const t of m.replay_pattern_tags ?? []) allTags.add(t);
  }
  if (!allTags.size) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h3 className="mb-2 font-semibold text-slate-200">Pattern groups</h3>
      <div className="flex flex-wrap gap-1">
        {[...allTags].map((tag) => (
          <button
            key={tag}
            type="button"
            className={`rounded px-2 py-0.5 text-xs ${
              filterTags.includes(tag)
                ? "bg-violet-700 text-violet-100"
                : "bg-slate-800 text-slate-400 hover:bg-slate-700"
            }`}
            onClick={() => toggleTag(tag)}
          >
            {TAG_LABELS[tag] ?? tag}
          </button>
        ))}
      </div>
      <ul className="mt-2 space-y-1 text-xs text-slate-400">
        {sweep.members.map((m, i) => (
          <li key={m.member_id}>
            <span className="text-slate-500">#{i}</span> {m.member_id}:{" "}
            {(m.replay_pattern_tags ?? []).map((t) => TAG_LABELS[t] ?? t).join(", ") || "—"}
          </li>
        ))}
      </ul>
    </section>
  );
}
