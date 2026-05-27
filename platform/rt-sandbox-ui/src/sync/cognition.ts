/** Tri-source sync cognition for RT↔Gazebo (PLAT-RT-G6). */

export type SyncSourceLabel =
  | "command_intent"
  | "adapter_feedback"
  | "sim_truth";

export function describeSyncSources(): { label: SyncSourceLabel; title: string; detail: string }[] {
  return [
    {
      label: "command_intent",
      title: "Command intent",
      detail: "Bridge registry — authoritative for spawn/move/delete",
    },
    {
      label: "adapter_feedback",
      title: "Adapter feedback",
      detail: "Pose sync mirror — explanatory drift vs command",
    },
    {
      label: "sim_truth",
      title: "Sim truth",
      detail: "Gazebo entity_state in live mode; mock memory in mock mode",
    },
  ];
}

export function adapterModeLabel(
  worldSummary: Record<string, unknown> | undefined,
): string {
  const mode = worldSummary?.adapter_mode;
  if (typeof mode === "string" && mode) return mode;
  return "off";
}

export function isAdapterSyncApplicable(
  worldSummary: Record<string, unknown> | undefined,
): boolean {
  const mode = adapterModeLabel(worldSummary);
  return mode === "mock" || mode === "live";
}

export function formatApplyLagMs(
  worldSummary: Record<string, unknown> | undefined,
): string | null {
  const lag = worldSummary?.apply_lag_ms;
  if (typeof lag === "number" && Number.isFinite(lag)) {
    return `${lag.toFixed(1)} ms`;
  }
  return null;
}

export type FeedbackEntityRow = {
  entityId: string;
  driftM: number | null;
  syncRevision: number | null;
};

export function feedbackEntityRows(
  worldSummary: Record<string, unknown> | undefined,
): FeedbackEntityRow[] {
  const raw = worldSummary?.feedback_entities;
  if (!Array.isArray(raw)) return [];
  return raw
    .map((item) => {
      if (!item || typeof item !== "object") return null;
      const row = item as Record<string, unknown>;
      const entityId = String(row.entity_id ?? "");
      if (!entityId) return null;
      const drift = row.drift_m;
      return {
        entityId,
        driftM: typeof drift === "number" ? drift : null,
        syncRevision:
          typeof row.sync_revision === "number" ? row.sync_revision : null,
      };
    })
    .filter((r): r is FeedbackEntityRow => r !== null);
}

export function shouldClearPendingReconcile(
  pending: boolean,
  worldSummary: Record<string, unknown> | undefined,
  lastCommandUtc: string | undefined,
): boolean {
  if (!pending) return false;
  const poll = worldSummary?.last_poll_utc;
  const cmd = worldSummary?.last_command_utc ?? lastCommandUtc;
  if (typeof poll !== "string" || typeof cmd !== "string") {
    return typeof poll === "string";
  }
  return poll >= cmd;
}
