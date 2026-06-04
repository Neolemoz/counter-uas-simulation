import type { RtIntelligenceAdvisoryTransportV1 } from "@/intelligence/intelligenceAdvisory";
import type { TelemetryChannel } from "./constants";

export type TelemetryPayloadByChannel = {
  session_health: Record<string, unknown>;
  lifecycle_state: Record<string, unknown>;
  world_summary: Record<string, unknown>;
  entity_pose_mirror: Record<string, unknown>;
  clock_mirror: Record<string, unknown>;
  tactical_state: Record<string, unknown>;
  tactical_recommendation: Record<string, unknown>;
  intelligence_advisory: RtIntelligenceAdvisoryTransportV1;
};

export interface TelemetryEvent {
  channel: string;
  session_id: string;
  timestamp_utc: string;
  payload: Record<string, unknown>;
  governance_banner?: string;
}

export interface ChannelSnapshot<C extends TelemetryChannel | undefined = undefined> {
  channel: C extends TelemetryChannel ? C : TelemetryChannel;
  payload: C extends TelemetryChannel ? TelemetryPayloadByChannel[C] : Record<string, unknown>;
  timestamp_utc: string;
  governance_banner?: string;
}

/** Latest event per channel from a drained batch (mirrors rt_telemetry_viz.py reversed scan). */
export function indexLatestByChannel(
  events: TelemetryEvent[],
): Partial<Record<TelemetryChannel, ChannelSnapshot>> {
  const out: Partial<Record<TelemetryChannel, ChannelSnapshot>> = {};
  for (const ev of events) {
    const ch = ev.channel as TelemetryChannel;
    out[ch] = {
      channel: ch,
      payload: ev.payload ?? {},
      timestamp_utc: ev.timestamp_utc,
      governance_banner: ev.governance_banner,
    };
  }
  return out;
}

/** Merge new snapshots over existing (newer batch wins per channel). */
export function mergeChannelSnapshots(
  existing: Partial<Record<TelemetryChannel, ChannelSnapshot>>,
  events: TelemetryEvent[],
): Partial<Record<TelemetryChannel, ChannelSnapshot>> {
  const incoming = indexLatestByChannel(events);
  return { ...existing, ...incoming };
}

export function entitiesFromSnapshot(
  snapshot: ChannelSnapshot | undefined,
): Array<Record<string, unknown>> {
  if (!snapshot) return [];
  const entities = snapshot.payload.entities;
  return Array.isArray(entities) ? (entities as Array<Record<string, unknown>>) : [];
}

export function sessionStateFromSnapshots(
  snapshots: Partial<Record<TelemetryChannel, ChannelSnapshot>>,
): string {
  const lifecycle = snapshots.lifecycle_state?.payload;
  if (lifecycle && typeof lifecycle.state === "string") {
    return lifecycle.state;
  }
  const health = snapshots.session_health?.payload;
  if (health && typeof health.state === "string") {
    return health.state;
  }
  return "unknown";
}
