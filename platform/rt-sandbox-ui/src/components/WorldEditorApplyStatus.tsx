export type ApplyRuntimeStatus =
  | { phase: "idle" }
  | { phase: "applying" }
  | { phase: "applied"; entityCount: number }
  | { phase: "failed"; message: string };

export const APPLY_STATUS_CLEAR_MS = 4000;

export function WorldEditorApplyStatus({ status }: { status: ApplyRuntimeStatus }) {
  if (status.phase === "idle") return null;

  const tone =
    status.phase === "applying"
      ? "text-amber-200"
      : status.phase === "applied"
        ? "text-emerald-200"
        : "text-red-300";

  let label = "";
  if (status.phase === "applying") {
    label = "Applying…";
  } else if (status.phase === "applied") {
    label = `Applied · ${status.entityCount} entit${status.entityCount === 1 ? "y" : "ies"}`;
  } else {
    label = `Failed · ${status.message}`;
  }

  return (
    <span
      className={`font-mono text-[11px] tabular-nums ${tone}`}
      data-testid="world-editor-apply-status"
      data-phase={status.phase}
    >
      {label}
    </span>
  );
}
