export type LiveMirrorFreshness = "fresh" | "stale" | "unavailable" | "unknown";

export type LiveTraceabilityFreshness =
  | "fresh"
  | "entity_stale"
  | "advisory_stale"
  | "both_stale"
  | "unknown";

export type LiveTraceabilityFreshnessInput = {
  mirrorFreshness: LiveMirrorFreshness | null | undefined;
  advisoryStale: boolean | null | undefined;
  advisoryStaleReason?: string | null;
};

export type LiveTraceabilityFreshnessResult = {
  freshness: LiveTraceabilityFreshness;
  trackStaleness: "fresh" | "stale" | "unknown";
  advisoryStale: boolean;
  advisoryStaleReason: string | null;
};

function isMirrorStale(mirrorFreshness: LiveMirrorFreshness | null | undefined): boolean {
  return mirrorFreshness === "stale" || mirrorFreshness === "unavailable";
}

export function deriveLiveTraceabilityFreshness({
  mirrorFreshness,
  advisoryStale,
  advisoryStaleReason = null,
}: LiveTraceabilityFreshnessInput): LiveTraceabilityFreshnessResult {
  const advisoryIsStale = advisoryStale === true;

  if (mirrorFreshness === "fresh" && !advisoryIsStale) {
    return {
      freshness: "fresh",
      trackStaleness: "fresh",
      advisoryStale: false,
      advisoryStaleReason: null,
    };
  }

  if (mirrorFreshness === "fresh" && advisoryIsStale) {
    return {
      freshness: "advisory_stale",
      trackStaleness: "fresh",
      advisoryStale: true,
      advisoryStaleReason,
    };
  }

  if (isMirrorStale(mirrorFreshness) && advisoryIsStale) {
    return {
      freshness: "both_stale",
      trackStaleness: "stale",
      advisoryStale: true,
      advisoryStaleReason,
    };
  }

  if (isMirrorStale(mirrorFreshness) && !advisoryIsStale) {
    return {
      freshness: "entity_stale",
      trackStaleness: "stale",
      advisoryStale: false,
      advisoryStaleReason: null,
    };
  }

  return {
    freshness: "unknown",
    trackStaleness: "unknown",
    advisoryStale: advisoryIsStale,
    advisoryStaleReason: advisoryIsStale ? advisoryStaleReason : null,
  };
}
