import type { CompareBadge } from "./experimentCompare";
import type { CompareStatusId } from "./compareStatusVocabulary";

export function pairwiseCompareStatus(badges: CompareBadge[]): CompareStatusId {
  if (badges.length === 0) return "aligned";
  return "divergent";
}

export function cellCompareStatus(hasValue: boolean, explanatory = false): CompareStatusId {
  if (explanatory) return "explanatory";
  if (!hasValue) return "missing";
  return "aligned";
}
