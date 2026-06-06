import { designateProtectedCenter } from "@/bridge/protectedCenterCommands";
import { isDesignatedProtectedCenter } from "@/cesium/defenseZoneVisualState";

export const REPLACE_PROTECTED_CENTER_SUFFIX =
  "Threat evaluation distances will use the new center.";

export function replaceProtectedCenterConfirmMessage(
  currentCenterId: string,
  nextEntityId: string,
): string {
  return (
    `Replace protected center "${currentCenterId}" with "${nextEntityId}"? ` +
    REPLACE_PROTECTED_CENTER_SUFFIX
  );
}

export interface ProtectedCenterDesignationAttempt {
  sessionId: string;
  entityId: string;
  replace: boolean;
}

export function resolveProtectedCenterDesignationAttempt(input: {
  sessionId: string | null;
  entityId: string | null;
  currentCenterId: string | null;
  editingEnabled: boolean;
  busy: boolean;
  confirm?: (message: string) => boolean;
}): ProtectedCenterDesignationAttempt | null {
  const { sessionId, entityId, currentCenterId, editingEnabled, busy } = input;
  if (!sessionId || !entityId || !editingEnabled || busy) {
    return null;
  }
  if (isDesignatedProtectedCenter(entityId, currentCenterId)) {
    return null;
  }

  if (
    currentCenterId != null &&
    currentCenterId !== entityId &&
    input.confirm?.(
      replaceProtectedCenterConfirmMessage(currentCenterId, entityId),
    ) === false
  ) {
    return null;
  }

  return {
    sessionId,
    entityId,
    replace: currentCenterId != null && currentCenterId !== entityId,
  };
}

export async function executeProtectedCenterDesignation(
  attempt: ProtectedCenterDesignationAttempt,
  designate = designateProtectedCenter,
): Promise<{ ok: true; entityId: string } | { ok: false; error: string }> {
  const result = await designate(
    attempt.sessionId,
    attempt.entityId,
    attempt.replace,
  );
  if (!result.ok) {
    return {
      ok: false,
      error: result.error_code ?? result.message ?? "designation failed",
    };
  }
  return { ok: true, entityId: attempt.entityId };
}
