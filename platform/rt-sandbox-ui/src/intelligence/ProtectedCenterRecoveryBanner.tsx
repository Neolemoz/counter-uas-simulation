import {
  protectedCenterRecoveryMessage,
  type ProtectedCenterClearReason,
} from "./protectedCenterCopy";

export function ProtectedCenterRecoveryBanner({
  recoveryNotice,
  protectedCenterEntityId,
}: {
  recoveryNotice: ProtectedCenterClearReason | null;
  protectedCenterEntityId: string | null;
}) {
  if (!recoveryNotice || protectedCenterEntityId) return null;

  return (
    <p
      className="rounded border border-amber-700/55 bg-amber-950/35 px-3 py-2 text-xs leading-relaxed text-amber-100"
      data-testid="protected-center-recovery-banner"
      data-recovery-reason={recoveryNotice}
    >
      {protectedCenterRecoveryMessage(recoveryNotice)}
    </p>
  );
}
