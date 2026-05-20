import { BoundingSphere, Cartesian3 } from "cesium";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { enuToCartographic } from "./coordinates";

function toCartesian(bundle: ReplaySaBundle, x: number, y: number, z = 0): Cartesian3 {
  const c = enuToCartographic(bundle, x, y, z);
  return Cartesian3.fromRadians(c.longitude, c.latitude, c.height);
}

export function collectReplayFitPoints(bundle: ReplaySaBundle): Cartesian3[] {
  const points: Cartesian3[] = [];
  for (const track of bundle.tracks) {
    for (const s of track.samples) {
      points.push(toCartesian(bundle, s.x_m, s.y_m, s.z_m ?? 0));
    }
  }
  for (const zone of bundle.zones) {
    const g = zone.geometry;
    if (g.type === "circle" && g.center_enu_m) {
      const [cx, cy, cz = 0] = g.center_enu_m;
      points.push(toCartesian(bundle, cx, cy, cz));
      if (g.radius_m) {
        points.push(toCartesian(bundle, cx + g.radius_m, cy, cz));
      }
    }
  }
  for (const ov of bundle.overlays) {
    for (const [x, y, z = 0] of ov.geometry.vertices_enu_m ?? []) {
      points.push(toCartesian(bundle, x, y, z));
    }
    for (const [x, y, z = 0] of ov.geometry.ridge_outline_enu_m ?? []) {
      points.push(toCartesian(bundle, x, y, z));
    }
  }
  for (const seg of bundle.los_segments ?? []) {
    for (const [x, y, z = 0] of seg.polyline_enu_m) {
      points.push(toCartesian(bundle, x, y, z));
    }
  }
  return points;
}

export function replayBoundingSphere(bundle: ReplaySaBundle): BoundingSphere | null {
  const points = collectReplayFitPoints(bundle);
  if (points.length === 0) return null;
  return BoundingSphere.fromPoints(points);
}

export function eventFocusPoints(
  bundle: ReplaySaBundle,
  lineIndex: number,
): Cartesian3[] {
  const points: Cartesian3[] = [];
  for (const track of bundle.tracks) {
    const sample = track.samples.find((s) => s.t === lineIndex);
    if (sample) {
      points.push(toCartesian(bundle, sample.x_m, sample.y_m, sample.z_m ?? 0));
    }
  }
  for (const ov of bundle.overlays) {
    const linked = ov.linked_event_ids ?? [];
    const ev = bundle.narrative.events.find(
      (e) => e.line_index === lineIndex && linked.includes(String(e.event_id)),
    );
    if (ev || linked.length === 0) {
      for (const [x, y, z = 0] of ov.geometry.vertices_enu_m ?? []) {
        points.push(toCartesian(bundle, x, y, z));
      }
    }
  }
  return points;
}
