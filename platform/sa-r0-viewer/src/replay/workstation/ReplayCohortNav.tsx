import { useCohortFilmstripStore } from "../cohortFilmstripStore";
import { useSweepStore } from "../useSweepStore";

type Props = {
  onOpenFilmstrip?: (indices: number[]) => void;
};

export function ReplayCohortNav({ onOpenFilmstrip }: Props) {
  const sweep = useSweepStore((s) => s.sweep);
  const activeCohortId = useSweepStore((s) => s.activeCohortId);
  const setActiveCohortId = useSweepStore((s) => s.setActiveCohortId);
  const setMemberIndex = useSweepStore((s) => s.setMemberIndex);
  const enterFilmstrip = useCohortFilmstripStore((s) => s.enterFilmstrip);

  if (!sweep?.replay_cohorts?.length) return null;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h3 className="mb-2 font-semibold text-slate-200">Replay cohorts</h3>
      <ul className="space-y-2">
        {sweep.replay_cohorts.map((c) => (
          <li key={c.cohort_id}>
            <button
              type="button"
              className={`w-full rounded px-2 py-1 text-left text-xs ${
                activeCohortId === c.cohort_id
                  ? "bg-violet-800/50 text-violet-100"
                  : "text-slate-300 hover:bg-slate-800"
              }`}
              onClick={() => {
                setActiveCohortId(c.cohort_id);
                const first = c.member_indices[0];
                if (first != null) setMemberIndex(first);
              }}
            >
              <span className="font-medium">{c.label}</span>
              <span className="block text-slate-500">{c.dominant_summary}</span>
            </button>
            {c.member_indices.length >= 2 && c.member_indices.length <= 4 && (
              <button
                type="button"
                className="mt-1 text-xs text-violet-400 hover:text-violet-200"
                onClick={() => {
                  if (onOpenFilmstrip) onOpenFilmstrip(c.member_indices);
                  else if (sweep) void enterFilmstrip(sweep, c.member_indices);
                }}
              >
                Open {c.member_indices.length}-member filmstrip
              </button>
            )}
          </li>
        ))}
      </ul>
      {activeCohortId && (
        <button
          type="button"
          className="mt-2 text-xs text-slate-500 hover:text-slate-300"
          onClick={() => setActiveCohortId(null)}
        >
          Clear cohort filter
        </button>
      )}
    </section>
  );
}
