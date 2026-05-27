import { StatusBadge } from "@/workstation/StatusBadge";
import { advisoryStateLabel, advisoryStateTone } from "./advisoryLabels";
import type { AdvisoryState } from "./advisoryTypes";

export function AdvisoryStateBadge({
  state,
  blocked,
  label,
}: {
  state: AdvisoryState | null;
  blocked: boolean;
  label?: string;
}) {
  const displayLabel = label ?? advisoryStateLabel(state);
  const tone = advisoryStateTone(state, blocked);
  const aria = `Advisory state: ${displayLabel} — not SA authority`;
  return (
    <span aria-label={aria}>
      <StatusBadge label={displayLabel} tone={tone} title={aria} />
    </span>
  );
}
