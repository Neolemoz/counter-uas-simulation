import type { CaptureStatusResponse } from "@/bridge/types";

export type LiveCaptureSummary = {
  captureActive: boolean;
  captureStatus: string;
  captureId: string | null;
  framesCount: number;
  entitiesCount: number;
  startedUtc: string | null;
};

export const INACTIVE_CAPTURE_SUMMARY: LiveCaptureSummary = {
  captureActive: false,
  captureStatus: "inactive",
  captureId: null,
  framesCount: 0,
  entitiesCount: 0,
  startedUtc: null,
};

export function shortCaptureId(id: string | null): string {
  if (!id) return "—";
  return id.length > 12 ? `${id.slice(0, 8)}…` : id;
}

export function formatCaptureStartedShort(startedUtc: string | null): string {
  if (!startedUtc) return "—";
  const match = startedUtc.match(/T(\d{2}:\d{2}:\d{2})/);
  if (match) return match[1];
  return startedUtc.length > 19 ? startedUtc.slice(11, 19) : startedUtc;
}

export function parseCaptureStatusFromResponse(
  resp: CaptureStatusResponse,
): LiveCaptureSummary {
  const captureActive =
    resp.capture_active === true || resp.capture_status === "active";
  return {
    captureActive,
    captureStatus:
      typeof resp.capture_status === "string"
        ? resp.capture_status
        : captureActive
          ? "active"
          : "inactive",
    captureId: typeof resp.capture_id === "string" ? resp.capture_id : null,
    framesCount:
      typeof resp.frames_count === "number" && Number.isFinite(resp.frames_count)
        ? resp.frames_count
        : 0,
    entitiesCount:
      typeof resp.entities_count === "number" &&
      Number.isFinite(resp.entities_count)
        ? resp.entities_count
        : 0,
    startedUtc: typeof resp.started_utc === "string" ? resp.started_utc : null,
  };
}
