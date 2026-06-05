export type LiveTrackMirrorFreshness = "fresh" | "stale" | "unavailable" | "unknown";

export type LiveTrackFreshness =
  | "fresh"
  | "entity_stale"
  | "advisory_stale"
  | "both_stale"
  | "unknown";

export type LiveTrackFreshnessInput = {
  mirrorFreshness: LiveTrackMirrorFreshness | null | undefined;
  advisoryStale: boolean | null | undefined;
  advisoryStaleReason?: string | null;
};

export type LiveTrackFreshnessResult = {
  freshness: LiveTrackFreshness;
  trackStaleness: "fresh" | "stale" | "unknown";
  advisoryStale: boolean;
  advisoryStaleReason: string | null;
};

function mirrorIsStale(mirrorFreshness: LiveTrackMirrorFreshness | null | undefined): boolean {
  return mirrorFreshness === "stale" || mirrorFreshness === "unavailable";
}

export function deriveLiveTrackFreshness({
  mirrorFreshness,
  advisoryStale,
  advisoryStaleReason = null,
}: LiveTrackFreshnessInput): LiveTrackFreshnessResult {
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

  if (mirrorIsStale(mirrorFreshness) && advisoryIsStale) {
    return {
      freshness: "both_stale",
      trackStaleness: "stale",
      advisoryStale: true,
      advisoryStaleReason,
    };
  }

  if (mirrorIsStale(mirrorFreshness) && !advisoryIsStale) {
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
