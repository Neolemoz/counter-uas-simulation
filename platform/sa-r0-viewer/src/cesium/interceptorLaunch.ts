import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { Position3 } from "@/replay/trackPlayback";

export function resolveInterceptorBase(
  bundle: ReplaySaBundle,
  trackId?: string,
): [number, number, number] | null {
  const bases = bundle.entities_static.filter((e) => e.kind === "interceptor_base");
  if (bases.length === 0) return null;
  if (trackId) {
    const prefix = trackId.replace(/_\d+$/, "");
    const match = bases.find(
      (b) => b.entity_id.includes(prefix) || b.entity_id.includes("int_base"),
    );
    if (match) return match.position_enu_m;
  }
  return bases[0].position_enu_m;
}

export function hasInterceptorSamples(bundle: ReplaySaBundle): boolean {
  const track = bundle.tracks.find((t) => t.role === "interceptor");
  return Boolean(track && track.samples.length > 0);
}

export function launchSegmentVisible(
  firstSampleT: number,
  currentT: number,
): boolean {
  return currentT >= firstSampleT;
}

export type LaunchSegment = {
  base: [number, number, number];
  firstSample: Position3;
  firstSampleT: number;
};

export function getLaunchSegment(
  bundle: ReplaySaBundle,
  currentT: number,
): LaunchSegment | null {
  const track = bundle.tracks.find((t) => t.role === "interceptor");
  if (!track || track.samples.length === 0) return null;
  const sorted = [...track.samples].sort((a, b) => a.t - b.t);
  const first = sorted[0];
  if (!launchSegmentVisible(first.t, currentT)) return null;
  const base = resolveInterceptorBase(bundle, track.track_id);
  if (!base) return null;
  return {
    base,
    firstSample: { x_m: first.x_m, y_m: first.y_m, z_m: first.z_m ?? 0, interpolated: false },
    firstSampleT: first.t,
  };
}
