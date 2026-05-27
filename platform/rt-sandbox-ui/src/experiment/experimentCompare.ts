import type { TacticalStatePayload } from "@/bridge/tacticalCommands";
import type { ExperimentRun } from "./experimentSchema";

export type CompareSource = "live" | "pinned" | "capture";

export type CompareSide = {
  label: string;
  sessionId: string;
  runId?: string;
  source: CompareSource;
  tactical: TacticalStatePayload | null;
  worldSummary: Record<string, unknown> | null;
  lifecycle: Record<string, unknown> | null;
  annexSummary: ExperimentRun["tactical_annex_summary"];
  terrainNote?: string;
};

export type CompareBadge = {
  id: string;
  label: string;
  detail?: string;
};

function tacticalFromRun(run: ExperimentRun | null): TacticalStatePayload | null {
  const raw = run?.snapshot?.tactical_state;
  if (!raw || typeof raw !== "object") return null;
  return raw as TacticalStatePayload;
}

export function sideFromPinnedRun(
  run: ExperimentRun,
  source: CompareSource = run.capture_candidate_id ? "capture" : "pinned",
): CompareSide {
  return {
    label: run.label,
    sessionId: run.session_id,
    runId: run.run_id,
    source,
    tactical: tacticalFromRun(run),
    worldSummary: (run.snapshot.world_summary as Record<string, unknown>) ?? null,
    lifecycle: (run.snapshot.lifecycle_state as Record<string, unknown>) ?? null,
    annexSummary: run.tactical_annex_summary ?? null,
    terrainNote: run.snapshot.terrain_context?.note,
  };
}

export function sideFromLive(
  label: string,
  sessionId: string,
  snapshots: {
    tactical_state?: Record<string, unknown>;
    world_summary?: Record<string, unknown>;
    lifecycle_state?: Record<string, unknown>;
  },
  terrainNote?: string,
): CompareSide {
  return {
    label,
    sessionId,
    source: "live",
    tactical: (snapshots.tactical_state as TacticalStatePayload) ?? null,
    worldSummary: snapshots.world_summary ?? null,
    lifecycle: snapshots.lifecycle_state ?? null,
    annexSummary: null,
    terrainNote,
  };
}

export function compareBadges(a: CompareSide, b: CompareSide): CompareBadge[] {
  const badges: CompareBadge[] = [];
  const ta = a.tactical;
  const tb = b.tactical;
  if ((ta?.tactical_mode ?? "") !== (tb?.tactical_mode ?? "")) {
    badges.push({
      id: "mode_changed",
      label: "mode_changed",
      detail: `${ta?.tactical_mode ?? "—"} → ${tb?.tactical_mode ?? "—"}`,
    });
  }
  const assignA = ta?.assigned_target_id ?? ta?.assigned_interceptor_id;
  const assignB = tb?.assigned_target_id ?? tb?.assigned_interceptor_id;
  if ((assignA ?? "") !== (assignB ?? "")) {
    badges.push({ id: "assignment_changed", label: "assignment_changed" });
  }
  if (
    typeof ta?.tti_s === "number" &&
    typeof tb?.tti_s === "number" &&
    Math.abs(ta.tti_s - tb.tti_s) > 0.05
  ) {
    badges.push({
      id: "tti_delta",
      label: "tti_delta",
      detail: `${ta.tti_s.toFixed(1)}s vs ${tb.tti_s.toFixed(1)}s`,
    });
  }
  if ((ta?.autonomous_loop_status ?? "") !== (tb?.autonomous_loop_status ?? "")) {
    badges.push({ id: "pause_resume_delta", label: "pause_resume_delta" });
  }
  return badges;
}

export function entityCount(world: Record<string, unknown> | null): number {
  if (!world) return 0;
  const n = world.entity_count;
  return typeof n === "number" ? n : 0;
}
