import type { CaptureHandoffRow } from "@/bridge/types";
import { advisoryInputFromMirrorRow } from "./advisoryInputFromMirrorRow";
import { deriveAdvisoryState } from "./deriveAdvisoryState";
import type { AdvisoryState } from "./advisoryTypes";

export function advisoryCountsSummary(
  rows: CaptureHandoffRow[],
  sessionLifecycleState?: string | null,
): string {
  const counts = new Map<string, number>();
  for (const row of rows) {
    const status = deriveAdvisoryState(
      advisoryInputFromMirrorRow(row, sessionLifecycleState),
    );
    const key = status.terminal
      ? "committed"
      : status.blocked
        ? "blocked"
        : status.advisory_state ?? "pending";
    counts.set(key, (counts.get(key) ?? 0) + 1);
  }
  if (counts.size === 0) return "";
  return [...counts.entries()]
    .map(([state, n]) => `${n} ${state.replace(/_/g, " ")}`)
    .join(" · ");
}

export function deriveAdvisoryForRow(
  row: CaptureHandoffRow,
  sessionLifecycleState?: string | null,
  options?: { poseAttested?: boolean },
) {
  return deriveAdvisoryState(
    advisoryInputFromMirrorRow(row, sessionLifecycleState, options),
  );
}

export function formatAdvisoryStateKey(state: AdvisoryState | null, terminal?: string): string {
  if (terminal) return "committed";
  return state ?? "pending";
}
