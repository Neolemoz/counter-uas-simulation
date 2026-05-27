/** Capture/handoff workflow cognition (PLAT-RT-T4) — explanatory only. */

export type CaptureReadinessTone = "ok" | "warn" | "neutral" | "error";

export interface CaptureReadiness {
  label: string;
  detail: string;
  tone: CaptureReadinessTone;
}

export interface PipelineStep {
  id: string;
  phase: string;
  title: string;
  cli: string;
  note: string;
}

export const HANDOFF_DOC_CHIPS = [
  "handoff_ready",
  "handoff_reviewed",
  "handoff_rejected",
  "handoff_import_deferred",
  "handoff_import_prepared",
  "handoff_import_committed",
] as const;

export const CAPTURE_PIPELINE_STEPS: PipelineStep[] = [
  {
    id: "capture",
    phase: "A",
    title: "Capture",
    cli: "capture_session (bridge / maintainer)",
    note: "Writes staging under runs/rt_sandbox/captures/ — not SA lineage.",
  },
  {
    id: "normalize",
    phase: "B",
    title: "Normalize",
    cli: "scripts/rt/rt_capture_normalize.py",
    note: "normalization_status must reach normalized before approval.",
  },
  {
    id: "review",
    phase: "C",
    title: "Handoff review",
    cli: "scripts/rt/rt_handoff_review.py",
    note: "Maintainer checklist — handoff_ready / defer / reject.",
  },
  {
    id: "approve",
    phase: "D",
    title: "Approve",
    cli: "scripts/rt/rt_capture_approve.py",
    note: "Produces approval + conversion manifests.",
  },
  {
    id: "prepare",
    phase: "E",
    title: "SA import prepare",
    cli: "scripts/rt/rt_sa_import.py prepare",
    note: "Handoff manifest under sa_handoff/ — still not corpus authority.",
  },
  {
    id: "import",
    phase: "F",
    title: "SA import commit",
    cli: "scripts/rt/rt_sa_import.py commit --corpus-dest",
    note: "Explicit maintainer commit — SA lineage begins here only.",
  },
];

/** Derive session capture readiness from lifecycle (rt_capture_continuity_v1). */
export function captureReadinessFromLifecycle(
  sessionState: string,
  connected: boolean,
): CaptureReadiness {
  if (!connected) {
    return {
      label: "No active session",
      detail: "Connect to correlate capture workflow with a session_id.",
      tone: "neutral",
    };
  }

  switch (sessionState) {
    case "stopped":
      return {
        label: "Capture eligible",
        detail:
          "Session stopped — maintainer may run capture_session via bridge CLI (not from browser).",
        tone: "ok",
      };
    case "captured":
      return {
        label: "Session captured",
        detail:
          "Capture candidate emitted for this session — continue normalize/review via maintainer CLIs.",
        tone: "ok",
      };
    case "running":
      return {
        label: "Active — stop before capture",
        detail: "Stop session before capture_session (capture only from stopped).",
        tone: "warn",
      };
    case "paused":
      return {
        label: "Paused — stop before capture",
        detail: "Resume or stop session; capture requires stopped state.",
        tone: "warn",
      };
    case "failed":
    case "runtime_crashed":
    case "cleanup_pending":
      return {
        label: "Not capture-safe",
        detail: "Failed or cleanup session — do not produce importable conversion manifests.",
        tone: "error",
      };
    case "discarded":
      return {
        label: "Session discarded",
        detail: "No capture artifacts expected from this session.",
        tone: "neutral",
      };
    case "created":
      return {
        label: "Starting",
        detail: "Wait for running or stop before capture planning.",
        tone: "neutral",
      };
    default:
      return {
        label: `Lifecycle: ${sessionState}`,
        detail: "Capture readiness is explanatory — staging truth is maintainer-CLI only.",
        tone: "neutral",
      };
  }
}
