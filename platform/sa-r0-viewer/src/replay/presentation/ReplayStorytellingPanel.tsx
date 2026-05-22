import type { ReplaySaBundle } from "../bundleSchema";
import { useSweepStore } from "../useSweepStore";
import { buildBundleStorytelling, buildSweepStorytelling } from "../cognition/replayStorytelling";
import { sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  bundle: ReplaySaBundle | null;
};

export function ReplayStorytellingPanel({ bundle }: Props) {
  const sweep = useSweepStore((s) => s.sweep);
  const sections = bundle ? buildBundleStorytelling(bundle) : [];
  const sweepSections = sweep ? buildSweepStorytelling(sweep) : [];
  const merged = [...sections];
  for (const s of sweepSections) {
    if (!merged.some((m) => m.id === s.id)) merged.push(s);
  }

  if (!merged.length) {
    return (
      <section className={`${sandboxSurfaces.panel} p-3 ${sandboxTypography.caption}`}>
        No storytelling sections available for this replay.
      </section>
    );
  }

  return (
    <section className={`${sandboxSurfaces.panel} p-3`}>
      <h2 className={`mb-1 ${sandboxTypography.sectionLabel} text-slate-200`}>Replay storytelling</h2>
      <p className={`mb-3 ${sandboxTypography.caption}`}>
        Deterministic replay-derived summaries — explanatory only, not causal proof.
      </p>
      <ul className="space-y-2.5">
        {merged.map((s) => (
          <li key={s.id} className={`${sandboxSurfaces.panelInset} p-2.5`}>
            <h3 className="mb-1 text-xs font-semibold uppercase tracking-wide text-amber-200/80">
              {s.title}
            </h3>
            <p className="text-xs leading-relaxed text-slate-300">{s.body}</p>
          </li>
        ))}
      </ul>
    </section>
  );
}
