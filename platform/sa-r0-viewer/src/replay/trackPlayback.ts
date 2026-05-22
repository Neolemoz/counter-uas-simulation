import type { z } from "zod";
import type { replaySaBundleSchema } from "./bundleSchema";

export type TrackSample = z.infer<typeof replaySaBundleSchema>["tracks"][number]["samples"][number];

export const INTERPOLATION_CAVEAT =
  "Interpolated between log-evidenced samples; not continuous path truth.";

export function sortedSamples(samples: TrackSample[]): TrackSample[] {
  return [...samples].sort((a, b) => a.t - b.t);
}

export type BracketResult =
  | { kind: "empty" }
  | { kind: "before"; at: TrackSample }
  | { kind: "after"; at: TrackSample }
  | { kind: "at"; at: TrackSample }
  | { kind: "between"; prev: TrackSample; next: TrackSample };

export function getBracketingSamples(samples: TrackSample[], t: number): BracketResult {
  const sorted = sortedSamples(samples);
  if (sorted.length === 0) return { kind: "empty" };
  if (t <= sorted[0].t) {
    return t === sorted[0].t ? { kind: "at", at: sorted[0] } : { kind: "before", at: sorted[0] };
  }
  const last = sorted[sorted.length - 1];
  if (t >= last.t) {
    return t === last.t ? { kind: "at", at: last } : { kind: "after", at: last };
  }
  for (let i = 0; i < sorted.length - 1; i++) {
    const prev = sorted[i];
    const next = sorted[i + 1];
    if (prev.t === t) return { kind: "at", at: prev };
    if (next.t === t) return { kind: "at", at: next };
    if (prev.t < t && t < next.t) return { kind: "between", prev, next };
  }
  return { kind: "at", at: last };
}

export type Position3 = { x_m: number; y_m: number; z_m: number; interpolated: boolean };

export function interpolatePosition(samples: TrackSample[], t: number): Position3 | null {
  const bracket = getBracketingSamples(samples, t);
  if (bracket.kind === "empty") return null;
  if (bracket.kind === "at" || bracket.kind === "before" || bracket.kind === "after") {
    const s = bracket.at;
    return { x_m: s.x_m, y_m: s.y_m, z_m: s.z_m ?? 0, interpolated: false };
  }
  const { prev, next } = bracket;
  const span = next.t - prev.t;
  const u = span > 0 ? (t - prev.t) / span : 0;
  return {
    x_m: prev.x_m + u * (next.x_m - prev.x_m),
    y_m: prev.y_m + u * (next.y_m - prev.y_m),
    z_m: (prev.z_m ?? 0) + u * ((next.z_m ?? 0) - (prev.z_m ?? 0)),
    interpolated: true,
  };
}

export type TrailFutureSplit = {
  trail: TrackSample[];
  future: TrackSample[];
  head: Position3 | null;
};

export function splitTrailAndFuture(samples: TrackSample[], t: number): TrailFutureSplit {
  const sorted = sortedSamples(samples);
  const trail = sorted.filter((s) => s.t <= t);
  const future = sorted.filter((s) => s.t > t);
  const head = interpolatePosition(samples, t);
  return { trail, future, head };
}
