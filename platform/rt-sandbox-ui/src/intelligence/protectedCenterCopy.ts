export const PROTECTED_CENTER_STATUS_BANNER =
  "Protected center — explicit designation only; enables distance-based threat evaluation.";

export const PROTECTED_CENTER_NONE_COPY = "No protected center designated.";

export const PROTECTED_CENTER_CLEARED_RESET_COPY =
  "Protected center designation cleared after reset.";

export const PROTECTED_CENTER_CLEARED_APPLY_COPY =
  "Protected center designation cleared after apply to runtime.";

export const PROTECTED_CENTER_REDESIGNATE_COPY =
  "Re-designate a protected center to restore threat evaluation.";

export type ProtectedCenterClearReason =
  | "reset_session"
  | "apply_scenario"
  | "protected_center_unavailable";

export const PROTECTED_CENTER_UNAVAILABLE_COPY =
  "Protected center designation is unavailable. Select an entity and choose Designate Protected Center to restore live threat evaluation.";

export const PROTECTED_CENTER_UNAVAILABLE_STRIP_COPY =
  "Protected center required — designate an entity to restore live threat evaluation.";

export const PROTECTED_CENTER_UNAVAILABLE_RECOVERY_BANNER_COPY =
  "Protected center designation is no longer valid. Select an entity and choose Designate Protected Center to restore live threat evaluation.";

export function isProtectedCenterUnavailable(
  staleReason: string | null | undefined,
): boolean {
  return staleReason === "protected_center_unavailable";
}

export function protectedCenterRecoveryMessage(
  reason: ProtectedCenterClearReason,
): string {
  switch (reason) {
    case "reset_session":
      return `${PROTECTED_CENTER_CLEARED_RESET_COPY} ${PROTECTED_CENTER_REDESIGNATE_COPY}`;
    case "apply_scenario":
      return `${PROTECTED_CENTER_CLEARED_APPLY_COPY} ${PROTECTED_CENTER_REDESIGNATE_COPY}`;
    case "protected_center_unavailable":
      return PROTECTED_CENTER_UNAVAILABLE_RECOVERY_BANNER_COPY;
  }
}
