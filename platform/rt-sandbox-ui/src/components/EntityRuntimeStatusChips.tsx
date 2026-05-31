import {
  runtimeStateChipClassName,
  runtimeStateChipLabel,
  runtimeStateChipTone,
} from "@/telemetry/entityMirrorUi";

function RuntimeStateChip({
  prefix,
  state,
  kind,
}: {
  prefix: string;
  state: string;
  kind: "target" | "assignment";
}) {
  const label = runtimeStateChipLabel(state);
  if (!label) return null;
  const tone = runtimeStateChipTone(state, kind);
  return (
    <span
      className={`inline-flex items-center rounded border px-1.5 py-0.5 font-mono text-[10px] font-semibold uppercase tracking-wide ${runtimeStateChipClassName(tone)}`}
    >
      {prefix} {label}
    </span>
  );
}

export function EntityRuntimeStatusChips({
  targetState,
  assignmentState,
}: {
  targetState: string | null;
  assignmentState: string | null;
}) {
  const hasTarget = Boolean(runtimeStateChipLabel(targetState));
  const hasAssignment = Boolean(runtimeStateChipLabel(assignmentState));
  if (!hasTarget && !hasAssignment) return null;

  return (
    <div className="flex flex-wrap items-center gap-1">
      {hasTarget && targetState && (
        <RuntimeStateChip prefix="target" state={targetState} kind="target" />
      )}
      {hasAssignment && assignmentState && (
        <RuntimeStateChip prefix="assign" state={assignmentState} kind="assignment" />
      )}
    </div>
  );
}
