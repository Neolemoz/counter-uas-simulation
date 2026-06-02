/**
 * Browser-only export helpers for RT layout → Monte Carlo job handoff (no execution).
 */

import type { McJobPreview } from "./mcJobPreview";
import { MC_JOB_PREVIEW_SCHEMA_VERSION } from "./mcJobPreview";
import type { RtLayoutScenarioV1 } from "./rtLayoutMcProfile";
import { LAYOUT_SCHEMA_VERSION } from "./rtLayoutMcProfile";

export type LayoutMcHandoffBundle = {
  schema_version: "rt_layout_mc_handoff_v1";
  layout: RtLayoutScenarioV1;
  mc_job_preview: McJobPreview;
  exported_utc: string;
};

export type ClipboardResult = { ok: true } | { ok: false; error: string };

function utcNow(): string {
  return new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
}

export function exportJson(value: unknown): string {
  return `${JSON.stringify(value, null, 2)}\n`;
}

export function enrichLayoutForHandoff(layout: RtLayoutScenarioV1): RtLayoutScenarioV1 {
  return {
    ...layout,
    created_utc: layout.created_utc ?? utcNow(),
    notes:
      layout.notes ??
      "Exported from RT Scenario Evaluation panel (browser handoff only; not SA authority).",
  };
}

export function exportRtLayoutScenarioJson(layout: RtLayoutScenarioV1): string {
  const enriched = enrichLayoutForHandoff(layout);
  if (enriched.schema_version !== LAYOUT_SCHEMA_VERSION) {
    throw new Error(`expected schema_version ${LAYOUT_SCHEMA_VERSION}`);
  }
  return exportJson(enriched);
}

export function exportMcJobPreviewJson(job: McJobPreview): string {
  if (job.schema_version !== MC_JOB_PREVIEW_SCHEMA_VERSION) {
    throw new Error(`expected schema_version ${MC_JOB_PREVIEW_SCHEMA_VERSION}`);
  }
  return exportJson(job);
}

export function buildLayoutMcHandoffBundle(
  layout: RtLayoutScenarioV1,
  job: McJobPreview,
): LayoutMcHandoffBundle {
  return {
    schema_version: "rt_layout_mc_handoff_v1",
    layout: enrichLayoutForHandoff(layout),
    mc_job_preview: job,
    exported_utc: utcNow(),
  };
}

export function exportLayoutMcHandoffBundleJson(
  layout: RtLayoutScenarioV1,
  job: McJobPreview,
): string {
  return exportJson(buildLayoutMcHandoffBundle(layout, job));
}

export function suggestedRtLayoutFilename(layout: RtLayoutScenarioV1): string {
  const safe = layout.layout_id.replace(/[^\w.-]+/g, "_").slice(0, 64);
  return `${safe || "rt_layout"}.json`;
}

export function suggestedMcJobPreviewFilename(job: McJobPreview): string {
  const safe = job.source_layout_id.replace(/[^\w.-]+/g, "_").slice(0, 48);
  return `${safe || "rt_layout"}_mc_job_preview.json`;
}

export function suggestedHandoffBundleFilename(layout: RtLayoutScenarioV1): string {
  const safe = layout.layout_id.replace(/[^\w.-]+/g, "_").slice(0, 48);
  return `${safe || "rt_layout"}_mc_handoff.json`;
}

export function isPreparedJobStale(
  preparedJob: McJobPreview | null | undefined,
  currentGeometryId: string | null | undefined,
): boolean {
  if (!preparedJob || !currentGeometryId) return false;
  return preparedJob.geometry_id !== currentGeometryId;
}

function triggerBrowserDownload(filename: string, json: string): void {
  const blob = new Blob([json], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

export function downloadRtLayoutScenario(layout: RtLayoutScenarioV1): void {
  triggerBrowserDownload(
    suggestedRtLayoutFilename(layout),
    exportRtLayoutScenarioJson(layout),
  );
}

export function downloadMcJobPreview(job: McJobPreview): void {
  triggerBrowserDownload(suggestedMcJobPreviewFilename(job), exportMcJobPreviewJson(job));
}

export function downloadLayoutMcHandoffBundle(
  layout: RtLayoutScenarioV1,
  job: McJobPreview,
): void {
  triggerBrowserDownload(
    suggestedHandoffBundleFilename(layout),
    exportLayoutMcHandoffBundleJson(layout, job),
  );
}

export async function copyTextToClipboard(text: string): Promise<ClipboardResult> {
  if (typeof navigator === "undefined" || !navigator.clipboard?.writeText) {
    return { ok: false, error: "clipboard unavailable" };
  }
  try {
    await navigator.clipboard.writeText(text);
    return { ok: true };
  } catch (err) {
    return {
      ok: false,
      error: err instanceof Error ? err.message : String(err),
    };
  }
}

export async function copyRtLayoutScenario(
  layout: RtLayoutScenarioV1,
): Promise<ClipboardResult> {
  return copyTextToClipboard(exportRtLayoutScenarioJson(layout));
}

export async function copyMcJobPreview(job: McJobPreview): Promise<ClipboardResult> {
  return copyTextToClipboard(exportMcJobPreviewJson(job));
}

export async function copyLayoutMcHandoffBundle(
  layout: RtLayoutScenarioV1,
  job: McJobPreview,
): Promise<ClipboardResult> {
  return copyTextToClipboard(exportLayoutMcHandoffBundleJson(layout, job));
}
