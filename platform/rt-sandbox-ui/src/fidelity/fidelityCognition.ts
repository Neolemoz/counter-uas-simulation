/** Fidelity truth cognition helpers (PLAT-RT-F5b P1). */

export type FidelityAttestationStatus = "available" | "stale" | "unavailable";

export type FidelityLabel =
  | "truth_attested"
  | "explanatory"
  | "command_authoritative";

export interface FidelityTruthSnapshot {
  schema?: string;
  session_id?: string;
  timestamp_utc?: string;
  governance_banner?: string;
  attestation_status?: string;
  entity_truth?: Array<{
    entity_id: string;
    entity_type?: string;
    sim_entity_ref?: string;
    truth_attested_pose?: Record<string, number>;
    sim_agl_m?: number | null;
  }>;
  los_truth?: { label?: string; pair_entity_ids?: string[] };
  dome_truth?: { sensor_id?: string; entities_in_nominal_dome?: number };
}

export interface FidelityContext {
  enableFidelityCoupling: boolean;
  fidelityAttestationStatus: FidelityAttestationStatus;
  fidelityLabel?: FidelityLabel;
  governanceBanner?: string;
  fidelityTruth?: FidelityTruthSnapshot;
  source?: string;
  authorityLabel?: string;
}

export interface PoseTruthDriftRow {
  entityId: string;
  driftM: number | null;
  commandPose?: Record<string, number>;
  truthPose?: Record<string, number>;
  simAglM?: number | null;
  flags: string[];
}

const DRIFT_THRESHOLD_M = 0.001;

export function extractFidelityContext(
  payload: Record<string, unknown> | undefined,
): FidelityContext {
  if (!payload) {
    return {
      enableFidelityCoupling: false,
      fidelityAttestationStatus: "unavailable",
    };
  }

  const coupling = payload.enable_fidelity_coupling === true;
  const rawStatus = payload.fidelity_attestation_status;
  const attestation: FidelityAttestationStatus =
    rawStatus === "available" || rawStatus === "stale" || rawStatus === "unavailable"
      ? rawStatus
      : "unavailable";

  const truthRaw = payload.fidelity_truth;
  const fidelityTruth =
    truthRaw && typeof truthRaw === "object"
      ? (truthRaw as FidelityTruthSnapshot)
      : undefined;

  const labelRaw = payload.fidelity_label;
  const fidelityLabel =
    labelRaw === "truth_attested" ||
    labelRaw === "explanatory" ||
    labelRaw === "command_authoritative"
      ? labelRaw
      : undefined;

  return {
    enableFidelityCoupling: coupling,
    fidelityAttestationStatus: coupling ? attestation : "unavailable",
    fidelityLabel,
    governanceBanner:
      typeof payload.governance_banner === "string"
        ? payload.governance_banner
        : undefined,
    fidelityTruth,
    source: typeof payload.source === "string" ? payload.source : undefined,
    authorityLabel:
      typeof payload.authority_label === "string"
        ? payload.authority_label
        : undefined,
  };
}

export function mergeFidelityContextFromPayloads(
  ...payloads: Array<Record<string, unknown> | undefined>
): FidelityContext {
  for (const payload of payloads) {
    const ctx = extractFidelityContext(payload);
    if (ctx.enableFidelityCoupling) return ctx;
  }
  return extractFidelityContext(payloads.find(Boolean));
}

export function isFidelityCouplingOn(ctx: FidelityContext): boolean {
  return ctx.enableFidelityCoupling;
}

export function fidelityHubLine(ctx: FidelityContext): string {
  if (!ctx.enableFidelityCoupling) {
    return "Fidelity: off (stub)";
  }
  return "Fidelity: truth_attested (sim)";
}

export function formatFidelityLabel(label: FidelityLabel | string): string {
  const map: Record<string, string> = {
    truth_attested: "truth_attested",
    explanatory: "explanatory",
    command_authoritative: "command_authoritative",
  };
  return map[label] ?? label;
}

export function truthFreshnessSummary(ctx: FidelityContext): string {
  if (!ctx.enableFidelityCoupling) {
    return "Fidelity coupling off — stub path only";
  }
  const ts = ctx.fidelityTruth?.timestamp_utc ?? "—";
  const att = ctx.fidelityAttestationStatus;
  const src = ctx.source ?? "unknown";
  const auth = ctx.authorityLabel ?? "unknown";
  return `truth ${att} · ${ts} · source ${src} · ${auth}`;
}

export function truthStaleBadge(
  ctx: FidelityContext,
): { label: string; tone: "warn" | "ok" } | null {
  if (!ctx.enableFidelityCoupling) return null;
  if (ctx.fidelityAttestationStatus === "stale") {
    return { label: "stale truth", tone: "warn" };
  }
  if (ctx.fidelityAttestationStatus === "unavailable") {
    return { label: "truth unavailable", tone: "warn" };
  }
  return null;
}

function poseDriftM(
  command: Record<string, unknown> | undefined,
  truth: Record<string, unknown> | undefined,
): number | null {
  if (!command || !truth) return null;
  const dx = Number(command.x ?? 0) - Number(truth.x ?? 0);
  const dy = Number(command.y ?? 0) - Number(truth.y ?? 0);
  const dz = Number(command.z ?? 0) - Number(truth.z ?? 0);
  if (![dx, dy, dz].every(Number.isFinite)) return null;
  return Math.hypot(dx, dy, dz);
}

export function poseTruthDriftRows(
  ctx: FidelityContext,
  worldSummary: Record<string, unknown> | undefined,
): PoseTruthDriftRow[] {
  if (!ctx.enableFidelityCoupling) return [];

  const registryById = new Map<string, Record<string, unknown>>();
  const entities = worldSummary?.entities;
  if (Array.isArray(entities)) {
    for (const ent of entities) {
      if (!ent || typeof ent !== "object") continue;
      const row = ent as Record<string, unknown>;
      const id = String(row.entity_id ?? "");
      if (id) registryById.set(id, row);
    }
  }

  const truthById = new Map<string, Record<string, unknown>>();
  for (const row of ctx.fidelityTruth?.entity_truth ?? []) {
    truthById.set(row.entity_id, row as unknown as Record<string, unknown>);
  }

  const allIds = new Set([...registryById.keys(), ...truthById.keys()]);
  const rows: PoseTruthDriftRow[] = [];

  for (const entityId of allIds) {
    const reg = registryById.get(entityId);
    const truthRow = truthById.get(entityId);
    const commandPose = reg?.pose as Record<string, unknown> | undefined;
    const truthPose = truthRow?.truth_attested_pose as
      | Record<string, unknown>
      | undefined;
    const flags: string[] = [];
    if ((reg && !truthRow) || (!reg && truthRow)) {
      flags.push("partial_truth");
    }
    const driftM = poseDriftM(commandPose, truthPose);
    if (driftM != null && driftM > DRIFT_THRESHOLD_M) {
      flags.push("pose_truth_drift");
    }
    rows.push({
      entityId,
      driftM,
      commandPose: commandPose as Record<string, number> | undefined,
      truthPose: truthPose as Record<string, number> | undefined,
      simAglM:
        typeof truthRow?.sim_agl_m === "number"
          ? (truthRow.sim_agl_m as number)
          : null,
      flags,
    });
  }

  return rows;
}

export function poseTruthDriftSummary(rows: PoseTruthDriftRow[]): {
  count: number;
  maxDriftM: number | null;
} {
  const driftRows = rows.filter(
    (r) => r.driftM != null && r.driftM > DRIFT_THRESHOLD_M,
  );
  if (driftRows.length === 0) {
    return { count: 0, maxDriftM: null };
  }
  const maxDriftM = Math.max(...driftRows.map((r) => r.driftM ?? 0));
  return { count: driftRows.length, maxDriftM };
}

export function losTruthSummary(ctx: FidelityContext): string | null {
  if (!ctx.enableFidelityCoupling || !ctx.fidelityTruth?.los_truth) return null;
  const los = ctx.fidelityTruth.los_truth;
  const label = los.label ?? "unknown";
  const pairs = (los.pair_entity_ids ?? []).join(", ") || "—";
  return `LOS truth: ${label} (${pairs})`;
}

export function domeTruthSummary(ctx: FidelityContext): string | null {
  if (!ctx.enableFidelityCoupling || !ctx.fidelityTruth?.dome_truth) return null;
  const dome = ctx.fidelityTruth.dome_truth;
  return `Dome truth: ${dome.sensor_id ?? "sensor"} · ${dome.entities_in_nominal_dome ?? 0} entities`;
}

export function losDivergenceBadge(
  ctx: FidelityContext,
  heuristicLabel: string | null | undefined,
): { label: string; tone: "warn" } | null {
  if (!ctx.enableFidelityCoupling || !ctx.fidelityTruth?.los_truth?.label) {
    return null;
  }
  const truthLabel = ctx.fidelityTruth.los_truth.label;
  if (!heuristicLabel || heuristicLabel === truthLabel) return null;
  return { label: "cognition_truth_divergence", tone: "warn" };
}

export function partialTruthFlags(
  ctx: FidelityContext,
  worldSummary?: Record<string, unknown>,
): string[] {
  const flags = new Set<string>();
  for (const row of poseTruthDriftRows(ctx, worldSummary)) {
    if (row.flags.includes("partial_truth")) flags.add("partial_truth");
  }
  if (ctx.enableFidelityCoupling && !ctx.fidelityTruth) {
    flags.add("partial_truth");
  }
  return [...flags];
}

export function simAglForEntity(
  ctx: FidelityContext,
  entityId: string,
): number | null {
  if (!ctx.enableFidelityCoupling || !ctx.fidelityTruth) return null;
  const row = (ctx.fidelityTruth.entity_truth ?? []).find(
    (e) => e.entity_id === entityId,
  );
  return typeof row?.sim_agl_m === "number" ? row.sim_agl_m : null;
}

export function backgroundFidelityLabel(ctx: FidelityContext): string {
  if (!ctx.enableFidelityCoupling) return "off";
  if (ctx.fidelityAttestationStatus === "stale") return "on · stale";
  if (ctx.fidelityAttestationStatus === "unavailable") return "on · unavailable";
  return "on";
}

export function selectedEntityPoseTruthReadout(
  ctx: FidelityContext,
  entityId: string | null | undefined,
  worldSummary: Record<string, unknown> | undefined,
): string | null {
  if (!ctx.enableFidelityCoupling || !entityId) return null;
  const row = poseTruthDriftRows(ctx, worldSummary).find(
    (r) => r.entityId === entityId,
  );
  if (!row?.commandPose || !row.truthPose) return null;
  const cmd = row.commandPose;
  const truth = row.truthPose;
  const drift =
    row.driftM != null ? ` · drift ${row.driftM.toFixed(3)} m` : "";
  return (
    `command (${cmd.x?.toFixed(1)}, ${cmd.y?.toFixed(1)}, ${cmd.z?.toFixed(1)})` +
    ` vs truth (${truth.x?.toFixed(1)}, ${truth.y?.toFixed(1)}, ${truth.z?.toFixed(1)})${drift}`
  );
}
