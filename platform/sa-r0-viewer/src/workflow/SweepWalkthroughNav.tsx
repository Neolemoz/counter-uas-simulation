import { useSweepStore } from "@/replay/useSweepStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { applySweepWalkthroughStep } from "@/navigation/experimentNavigation";
import { sandboxSurfaces, sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  hooks: ExperimentNavHooks;
};

export function SweepWalkthroughNav({ hooks }: Props) {
  const sweep = useSweepStore((s) => s.sweep);
  const steps = sweep?.presentation_walkthrough?.steps ?? [];
  if (steps.length === 0) return null;

  return (
    <section className={`${sandboxSurfaces.panel} border-violet-900/30 p-2.5`}>
      <h3 className={`mb-1 ${sandboxTypography.sectionLabel} text-violet-200/80`}>
        Walkthrough steps
      </h3>
      <p className={`mb-2 ${sandboxTypography.caption}`}>
        Derived sweep presentation — navigation only, not live execution.
      </p>
      <ul className="space-y-1">
        {steps.map((step) => (
          <li key={step.step_id}>
            <button
              type="button"
              className="w-full rounded-md border border-slate-700/60 bg-slate-900/50 px-2 py-1.5 text-left text-xs text-slate-200 hover:bg-slate-800/60"
              onClick={() => void applySweepWalkthroughStep(step, hooks)}
            >
              <span className="text-slate-500">[{step.kind}]</span> {step.label}
            </button>
          </li>
        ))}
      </ul>
    </section>
  );
}
