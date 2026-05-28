import { compareModeCoachLine } from "./compareModeCoach";
import type { CompareModeId } from "./experimentUnifiedReview";

export function ExperimentCompareModeCoach({ mode }: { mode: CompareModeId }) {
  return (
    <p className="text-[10px] text-slate-400" data-testid="compare-mode-coach">
      {compareModeCoachLine(mode)}
    </p>
  );
}
