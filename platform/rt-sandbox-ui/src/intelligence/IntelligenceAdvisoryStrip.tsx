import { advisoryUiState, type RtIntelligenceAdvisoryTransportV1 } from "./intelligenceAdvisory";
import {
  INTELLIGENCE_ADVISORY_BANNER,
  INTELLIGENCE_ADVISORY_SHORT_COPY,
  INTELLIGENCE_ADVISORY_STALE_COPY,
} from "./intelligenceGovernance";
import {
  isProtectedCenterUnavailable,
  PROTECTED_CENTER_UNAVAILABLE_STRIP_COPY,
} from "./protectedCenterCopy";
import {
  getAdvisoryConfidenceLabel,
  getTopAdvisory,
} from "./intelligenceSelectors";

function advisoryCountLabel(count: number): string {
  return count === 1 ? "1 advisory" : `${count} advisories`;
}

function formatTopLine(transport: RtIntelligenceAdvisoryTransportV1): string {
  const top = getTopAdvisory(transport);
  if (!top) return "No current intelligence advisories.";
  const rank = top.threat_evaluation.threat_rank;
  const defender = top.recommended_defender.defender_id ?? "no feasible defender";
  const rankLabel = rank == null ? "unranked" : `#${rank}`;
  return `${rankLabel} ${top.identity.attacker_id} -> ${defender} · ${getAdvisoryConfidenceLabel(top)}`;
}

export function IntelligenceAdvisoryStrip({
  transport,
}: {
  transport: RtIntelligenceAdvisoryTransportV1 | null | undefined;
}) {
  const state = advisoryUiState(transport);
  const count = transport?.advisories.length ?? 0;
  const line =
    state === "loading"
      ? "Waiting for advisory transport."
      : state === "stale"
        ? isProtectedCenterUnavailable(transport?.stale_reason)
          ? PROTECTED_CENTER_UNAVAILABLE_STRIP_COPY
          : INTELLIGENCE_ADVISORY_STALE_COPY
        : state === "empty" || !transport
          ? "No current intelligence advisories."
          : formatTopLine(transport);

  return (
    <section
      className="rounded border border-indigo-800/50 bg-indigo-950/25 px-2.5 py-1.5 text-[10px] text-indigo-100"
      data-testid={
        state === "stale" && isProtectedCenterUnavailable(transport?.stale_reason)
          ? "intelligence-advisory-strip-protected-center-unavailable"
          : "intelligence-advisory-strip"
      }
    >
      <div className="flex flex-wrap items-center justify-between gap-2">
        <span>{line}</span>
        <span className="text-indigo-200/70">
          {advisoryCountLabel(count)}
        </span>
      </div>
      <p className="mt-1 text-amber-100/80" title={INTELLIGENCE_ADVISORY_BANNER}>
        {INTELLIGENCE_ADVISORY_SHORT_COPY}
      </p>
    </section>
  );
}
