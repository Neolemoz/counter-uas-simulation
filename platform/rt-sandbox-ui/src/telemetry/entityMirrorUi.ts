export type RuntimeStateChipTone = "neutral" | "active" | "assigned" | "idle" | "warn";

export function runtimeStateChipLabel(state: string | null | undefined): string | null {
  if (!state?.trim()) return null;
  const normalized = state.trim().replace(/_/g, " ").toUpperCase();
  if (normalized === "NONE") return null;
  return normalized;
}

export function runtimeStateChipTone(
  state: string | null | undefined,
  kind: "target" | "assignment",
): RuntimeStateChipTone {
  const normalized = state?.trim().toLowerCase() ?? "";
  if (!normalized || normalized === "none" || normalized === "idle" || normalized === "cleared") {
    return "idle";
  }
  if (normalized === "assigned") {
    return kind === "assignment" ? "assigned" : "assigned";
  }
  if (normalized === "tracking") return "active";
  if (normalized === "engaged" || normalized === "locked") return "warn";
  return "neutral";
}

export function runtimeStateChipClassName(tone: RuntimeStateChipTone): string {
  switch (tone) {
    case "assigned":
      return "border-emerald-700/60 bg-emerald-950/70 text-emerald-100";
    case "active":
      return "border-cyan-700/60 bg-cyan-950/70 text-cyan-100";
    case "idle":
      return "border-slate-700 bg-slate-900/80 text-slate-400";
    case "warn":
      return "border-amber-700/60 bg-amber-950/70 text-amber-100";
    default:
      return "border-slate-600 bg-slate-900/70 text-slate-300";
  }
}

export function compactTargetStateChip(
  targetState: string | null | undefined,
): string | null {
  const label = runtimeStateChipLabel(targetState);
  if (!label) return null;
  if (label === "ASSIGNED") return "TGT";
  if (label === "TRACKING") return "TRK";
  if (label.length <= 5) return label;
  return label.slice(0, 4);
}
