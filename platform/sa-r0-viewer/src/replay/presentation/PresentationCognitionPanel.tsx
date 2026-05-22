import { useClockStore } from "../clockStore";
import { useSweepStore } from "../useSweepStore";
import { computeCognitionIndicators } from "../cognition/replayStorytelling";
import { sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

export function PresentationCognitionPanel() {
  const bundle = useClockStore((s) => s.bundle);
  const sweep = useSweepStore((s) => s.sweep);
  const indicators = computeCognitionIndicators(bundle, sweep);

  return (
    <section className={`${sandboxSurfaces.panel} p-3`}>
      <h2 className={`mb-1 ${sandboxTypography.sectionLabel} text-slate-200`}>
        Review orientation
      </h2>
      <p className={`mb-2 ${sandboxTypography.caption}`}>
        Descriptive tags for mentor walkthrough — not scores or rankings.
      </p>
      <ul className="flex flex-wrap gap-1.5">
        {Object.entries(indicators).map(([key, label]) => (
          <li
            key={key}
            className="rounded-md border border-slate-700/60 bg-slate-950/50 px-2 py-1 text-[11px] text-slate-400"
          >
            <span className="text-slate-500">{key.replace(/([A-Z])/g, " $1").trim()}: </span>
            {label}
          </li>
        ))}
      </ul>
    </section>
  );
}
