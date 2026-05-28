export type CompareStatusId =
  | "aligned"
  | "divergent"
  | "missing"
  | "explanatory"
  | "not_comparable";

export const COMPARE_STATUS_LABELS: Record<CompareStatusId, string> = {
  aligned: "aligned",
  divergent: "divergent",
  missing: "missing",
  explanatory: "explanatory",
  not_comparable: "not comparable",
};

export function formatCompareStatus(id: CompareStatusId): string {
  return COMPARE_STATUS_LABELS[id];
}

export function deriveRowCompareStatus(
  primary: string,
  secondary: string,
  options?: { explanatory?: boolean },
): CompareStatusId {
  if (options?.explanatory) return "explanatory";
  const p = primary.trim();
  const s = secondary.trim();
  if (p === "—" || s === "—" || p === "" || s === "") {
    if (p === "—" && s === "—") return "missing";
    return "missing";
  }
  if (p === s) return "aligned";
  return "divergent";
}
