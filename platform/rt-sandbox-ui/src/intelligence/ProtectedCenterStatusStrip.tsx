import type { UiEntity } from "@/editing/localEntityMirror";
import { ENTITY_LABELS, type EntityType } from "@/world/entityCatalog";
import {
  PROTECTED_CENTER_NONE_COPY,
  PROTECTED_CENTER_STATUS_BANNER,
} from "./protectedCenterCopy";
import {
  ProtectedCenterRecoveryBanner,
} from "./ProtectedCenterRecoveryBanner";
import type { ProtectedCenterClearReason } from "./protectedCenterCopy";

function entityTypeLabel(entityType: string | undefined): string {
  if (!entityType) return "unknown";
  return ENTITY_LABELS[entityType as EntityType] ?? entityType;
}

export function ProtectedCenterStatusStrip({
  protectedCenterEntityId,
  protectedCenterRecoveryNotice = null,
  entities,
}: {
  protectedCenterEntityId: string | null;
  protectedCenterRecoveryNotice?: ProtectedCenterClearReason | null;
  entities: UiEntity[];
}) {
  const designated = protectedCenterEntityId
    ? entities.find((ent) => ent.entity_id === protectedCenterEntityId)
    : null;

  return (
    <section
      className="space-y-2"
      data-testid="protected-center-status-region"
    >
      <ProtectedCenterRecoveryBanner
        recoveryNotice={protectedCenterRecoveryNotice}
        protectedCenterEntityId={protectedCenterEntityId}
      />
      <div
        className="rounded border border-emerald-800/45 bg-emerald-950/20 px-3 py-2 text-xs"
        data-testid="protected-center-status-strip"
      >
      <p className="text-[10px] text-emerald-100/80">{PROTECTED_CENTER_STATUS_BANNER}</p>
      <div className="mt-1 flex flex-wrap items-center justify-between gap-2">
        <span className="font-semibold uppercase tracking-wide text-emerald-200/90">
          Protected center
        </span>
        {protectedCenterEntityId ? (
          <span className="font-mono text-emerald-100" data-testid="protected-center-designated">
            {entityTypeLabel(designated?.entity_type)} · {protectedCenterEntityId}
          </span>
        ) : (
          <span className="text-slate-400" data-testid="protected-center-none">
            {PROTECTED_CENTER_NONE_COPY}
          </span>
        )}
      </div>
    </div>
    </section>
  );
}
