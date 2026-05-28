import type { BlockerGroupId } from "./advisoryTypes";

const LABELS: Record<BlockerGroupId, string> = {
  normalization: "normalize",
  review_attestation: "review",
  approval_gate: "approve",
  packaging: "package",
  lineage: "lineage",
  experiment_warn: "exp warn",
  terminal_block: "blocked",
};

export function AdvisoryBlockerGroupChips({
  groups,
  max = 3,
}: {
  groups: BlockerGroupId[];
  max?: number;
}) {
  if (!groups.length) return null;
  const visible = groups.slice(0, max);
  const rest = groups.length - visible.length;
  return (
    <span className="inline-flex flex-wrap gap-1">
      {visible.map((g) => (
        <span
          key={g}
          className="rounded border border-slate-600 bg-slate-900 px-1.5 py-0.5 text-[10px] text-slate-400"
          title={`Blocker group: ${g}`}
        >
          {LABELS[g] ?? g}
        </span>
      ))}
      {rest > 0 && (
        <span className="text-[10px] text-slate-500">+{rest}</span>
      )}
    </span>
  );
}
