export type AuthorityLabel =
  | "command_authoritative"
  | "explanatory_sync"
  | "explanatory_telemetry"
  | "replay_boundary_scoped"
  | string;

export type HealthStatus = "ok" | "stale" | "mismatch" | "feedback_lost" | string;

export interface HealthBadge {
  label: string;
  tone: "ok" | "warn" | "error" | "neutral";
}

const AUTHORITY_DESCRIPTIONS: Record<string, string> = {
  command_authoritative: "Bridge EntityRegistry command truth",
  truth_attested: "Sim-scoped attestation when fidelity coupling is on",
  explanatory_sync: "PoseSyncMirror drift/stale evidence only",
  explanatory_telemetry: "TelemetryMirror read model; not replay authority",
  replay_boundary_scoped: "RT staging artifact; SA import requires approval",
};

const SOURCE_DESCRIPTIONS: Record<string, string> = {
  bridge_session: "Bridge session runtime",
  bridge_registry: "Bridge entity registry",
  adapter_feedback: "Adapter pose feedback mirror",
  adapter_telemetry: "Adapter telemetry poll mirror",
};

export function describeAuthority(label: AuthorityLabel | undefined): string {
  if (!label) return "Authority label unavailable";
  return AUTHORITY_DESCRIPTIONS[label] ?? `Unknown authority: ${label}`;
}

export function describeSource(source: string | undefined): string {
  if (!source) return "Source unavailable";
  return SOURCE_DESCRIPTIONS[source] ?? source;
}

export function healthBadge(
  syncHealth: HealthStatus | undefined,
  telemetryHealth: HealthStatus | undefined,
): HealthBadge[] {
  const badges: HealthBadge[] = [];
  if (syncHealth) {
    badges.push({
      label: `sync: ${syncHealth}`,
      tone:
        syncHealth === "ok"
          ? "ok"
          : syncHealth === "mismatch"
            ? "error"
            : "warn",
    });
  }
  if (telemetryHealth) {
    badges.push({
      label: `telemetry: ${telemetryHealth}`,
      tone:
        telemetryHealth === "ok"
          ? "ok"
          : telemetryHealth === "feedback_lost"
            ? "error"
            : "warn",
    });
  }
  return badges;
}

export function isStaleTelemetry(payload: Record<string, unknown>): boolean {
  return payload.telemetry_health === "stale";
}

export function isStaleSync(payload: Record<string, unknown>): boolean {
  const sync = payload.sync_health;
  return sync !== undefined && sync !== "ok";
}

export function governanceCopyForPayload(
  payload: Record<string, unknown>,
): string | undefined {
  const banner = payload.governance_banner;
  return typeof banner === "string" ? banner : undefined;
}

export function formatAuthorityChip(authorityLabel: string): string {
  const short: Record<string, string> = {
    command_authoritative: "cmd auth",
    truth_attested: "sim truth",
    explanatory_sync: "sync mirror",
    explanatory_telemetry: "telemetry mirror",
    replay_boundary_scoped: "replay boundary",
  };
  return short[authorityLabel] ?? authorityLabel;
}

export function formatHealthChip(label: string): string {
  return label.replace(/^sync: /, "sync ").replace(/^telemetry: /, "tel ");
}

export function sessionContextLine(
  sessionId: string | null | undefined,
  role: "active" | "background" = "active",
): string | null {
  if (!sessionId) return null;
  const short = sessionId.length > 8 ? sessionId.slice(0, 8) : sessionId;
  return `session ${short} (${role}) — correlation only; not lineage`;
}

export function cognitionSummary(payload: Record<string, unknown>): {
  source: string;
  authorityLabel: string;
  authorityDescription: string;
  sourceDescription: string;
  badges: HealthBadge[];
  stale: boolean;
  governanceBanner?: string;
} {
  const source =
    typeof payload.source === "string" ? payload.source : "unknown";
  const authorityLabel =
    typeof payload.authority_label === "string"
      ? payload.authority_label
      : "unknown";
  const syncHealth =
    typeof payload.sync_health === "string" ? payload.sync_health : undefined;
  const telemetryHealth =
    typeof payload.telemetry_health === "string"
      ? payload.telemetry_health
      : undefined;

  return {
    source,
    authorityLabel,
    authorityDescription: describeAuthority(authorityLabel),
    sourceDescription: describeSource(source),
    badges: healthBadge(syncHealth, telemetryHealth),
    stale: isStaleTelemetry(payload) || isStaleSync(payload),
    governanceBanner: governanceCopyForPayload(payload),
  };
}
