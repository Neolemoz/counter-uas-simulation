import type { ReplaySaBundle } from "./bundleSchema";
import { interpolatePosition } from "./trackPlayback";

export type OnboardPhase =
  | "no_samples"
  | "pre_launch"
  | "en_route"
  | "target_acquired"
  | "intercept_window";

const ACQUIRE_RANGE_M = 800;
const INTERCEPT_TGO_S = 5;

export function getOnboardPhase(bundle: ReplaySaBundle, currentT: number): OnboardPhase {
  const intTrack = bundle.tracks.find((t) => t.role === "interceptor");
  if (!intTrack || intTrack.samples.length === 0) return "no_samples";

  const sorted = [...intTrack.samples].sort((a, b) => a.t - b.t);
  const firstT = sorted[0].t;
  if (currentT < firstT) return "pre_launch";

  const hasSelectionEvent = bundle.narrative.events.some(
    (e) =>
      e.category === "selection" &&
      typeof e.line_index === "number" &&
      e.line_index <= currentT,
  );
  if (hasSelectionEvent) return "intercept_window";

  const telemetry = bundle.panels?.telemetry_series ?? [];
  const atT = telemetry.filter((row) => Number(row.t) <= currentT);
  const latest = atT.at(-1);
  if (latest?.t_go_s != null && Number(latest.t_go_s) <= INTERCEPT_TGO_S) {
    return "intercept_window";
  }

  const threatTrack = bundle.tracks.find((t) => t.role === "threat");
  const intPos = interpolatePosition(intTrack.samples, currentT);
  const threatPos = threatTrack
    ? interpolatePosition(threatTrack.samples, currentT)
    : null;
  if (intPos && threatPos) {
    const dx = intPos.x_m - threatPos.x_m;
    const dy = intPos.y_m - threatPos.y_m;
    const dz = intPos.z_m - threatPos.z_m;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    if (dist <= ACQUIRE_RANGE_M) return "target_acquired";
  }

  return "en_route";
}

export const ONBOARD_PHASE_COPY: Record<
  OnboardPhase,
  { title: string; detail: string }
> = {
  no_samples: {
    title: "No onboard samples",
    detail: "No interceptor trajectory samples in bundle.",
  },
  pre_launch: {
    title: "Pre-launch",
    detail: "Replay clock before first log-evidenced interceptor sample.",
  },
  en_route: {
    title: "Launched / en route",
    detail: "Interceptor motion derived from sparse replay samples only.",
  },
  target_acquired: {
    title: "Target acquired (mock)",
    detail: "Proximity band between interpolated threat and interceptor positions.",
  },
  intercept_window: {
    title: "Intercept window (mock)",
    detail: "Associated with selection event or low t_go in replay telemetry — not engagement authority.",
  },
};
