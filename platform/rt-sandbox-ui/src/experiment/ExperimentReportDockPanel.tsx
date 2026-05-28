import { useEffect, useMemo, useState } from "react";
import { REVIEW_PACKET_GOVERNANCE_BANNER } from "./reviewPacketSchema";
import { REPORT_DOCK_SLOTS } from "./experimentUnifiedReview";
import { formatImportError } from "./experimentImportGuards";
import {
  buildReviewPacketPreview,
  type ReportDockPresence,
} from "./reviewPacketPreview";
import {
  copyReviewPacketJson,
  downloadReviewPacket,
} from "./reviewPacketExport";
import { buildPacketSectionsPreview } from "./reviewPacketSections";
import { ReviewPacketSectionCard } from "./ReviewPacketSectionCard";
import { ReviewPacketGroupedSummary } from "./ReviewPacketGroupedSummary";
import {
  groupPacketSections,
  sectionOrganizationHint,
} from "./packetSectionGroups";
import {
  buildStepCompletionMap,
  resolveStepCompletion,
  slotIdToReviewStep,
} from "./reviewStepCompletion";
import { ReviewStepCompletionBadge } from "./ReviewStepCompletionBadge";
import {
  defaultGroupExpandedMap,
  REPORT_DOCK_GROUPS,
  type ReportDockGroupId,
} from "./reportDockGroups";
import { CompareStatusChip } from "./CompareStatusChip";
import { buildMultiManifestDiff } from "./multiManifestDiff";
import { summarizeMultiManifestDiffStatus } from "./multiManifestDiffColumns";
import { useReportDockPacketTab } from "./useReportDockPacketTab";
import type { ExperimentCohortIndex } from "./cohortSchema";
import type { ExperimentManifest } from "./experimentSchema";
import type { WorkbenchV2State } from "./workbenchV2State";

export type { ReportDockPresence };

const PREVIEW_MAX = 2000;

function truncateJson(text: string): string {
  if (text.length <= PREVIEW_MAX) return text;
  return `${text.slice(0, PREVIEW_MAX)}\n…`;
}

export function ExperimentReportDockPanel({
  v2State,
  manifest,
  presence,
  previews,
  onImportSlot,
  onExportSlot,
  packetTabFocusToken,
  cohortLabel,
  cohort,
}: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  previews: Partial<Record<string, string>>;
  onImportSlot: (slotId: string, text: string) => string | null;
  onExportSlot: (slotId: string) => string | null;
  packetTabFocusToken?: number;
  cohortLabel?: string | null;
  cohort?: ExperimentCohortIndex | null;
}) {
  const [importError, setImportError] = useState<string | null>(null);
  const [previewSlot, setPreviewSlot] = useState<string | null>(null);
  const {
    activeTab,
    setActiveTab,
    copyStatus,
    setCopyStatus,
    packetDownloaded,
    setPacketDownloaded,
    packetTabEverFocused,
    openPacketTab,
  } = useReportDockPacketTab(packetTabFocusToken);

  const [groupExpanded, setGroupExpanded] = useState<Record<ReportDockGroupId, boolean>>(
    () => defaultGroupExpandedMap(v2State.review_step),
  );

  useEffect(() => {
    setGroupExpanded(defaultGroupExpandedMap(v2State.review_step));
  }, [v2State.review_step]);

  const completionOptions = useMemo(
    () => ({
      v2State,
      manifest,
      presence,
      packetTabEverFocused,
      packetDownloaded,
    }),
    [v2State, manifest, presence, packetTabEverFocused, packetDownloaded],
  );

  const stepCompletion = useMemo(
    () => buildStepCompletionMap(completionOptions),
    [completionOptions],
  );

  const packetPreview = useMemo(
    () => buildReviewPacketPreview({ v2State, manifest, presence }),
    [v2State, manifest, presence],
  );

  const multiManifestStatusLine = useMemo(() => {
    if (v2State.compare_mode !== "multi_manifest_diff") return undefined;
    const rows = buildMultiManifestDiff({
      cohort: cohort ?? null,
      primaryManifestRef: v2State.primary_manifest_ref,
      secondaryManifestRef: v2State.secondary_manifest_ref,
      loadedManifest: manifest,
    });
    if (rows.length === 0) return undefined;
    return summarizeMultiManifestDiffStatus(rows);
  }, [v2State, cohort, manifest]);

  const packetJson = useMemo(
    () => JSON.stringify(packetPreview, null, 2),
    [packetPreview],
  );

  const packetSections = useMemo(
    () =>
      buildPacketSectionsPreview({
        v2State,
        manifest,
        presence,
        cohortLabel,
        stepCompletion,
        multiManifestStatusLine,
      }),
    [v2State, manifest, presence, cohortLabel, stepCompletion, multiManifestStatusLine],
  );

  const groupedSections = useMemo(
    () => groupPacketSections(packetSections),
    [packetSections],
  );

  const slotPresent: Record<string, boolean> = {
    f1_analytics: presence.f1_analytics,
    f3_annex: presence.f3_annex,
    f5_metrics: presence.f5_metrics,
    f5b_fidelity: presence.f5b_fidelity,
  };

  const importSlot = (slotId: string) => {
    const text = window.prompt(`Paste ${slotId} JSON`);
    if (!text) return;
    const err = onImportSlot(slotId, text);
    if (err) {
      setImportError(formatImportError(err));
      return;
    }
    setImportError(null);
    setPreviewSlot(slotId);
  };

  const handleCopyPacket = async () => {
    const result = await copyReviewPacketJson(packetPreview);
    if (result.ok) {
      setCopyStatus("copied");
    } else {
      setCopyStatus(result.error);
    }
  };

  const handleDownloadPacket = () => {
    downloadReviewPacket(packetPreview);
    setPacketDownloaded(true);
  };

  const slotById = useMemo(
    () => new Map(REPORT_DOCK_SLOTS.map((s) => [s.id, s])),
    [],
  );

  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="report-dock-panel"
    >
      <div className="flex gap-2">
        <button
          type="button"
          className={`text-xs ${activeTab === "slots" ? "text-cyan-300" : "text-slate-500"}`}
          onClick={() => setActiveTab("slots")}
        >
          Report slots
        </button>
        <button
          type="button"
          className={`text-xs ${activeTab === "packet" ? "text-cyan-300" : "text-slate-500"}`}
          onClick={openPacketTab}
          data-testid="report-dock-packet-tab"
        >
          Review packet
        </button>
      </div>
      {activeTab === "slots" && (
        <>
          <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
            Report dock
          </p>
          {importError && <p className="text-xs text-red-400">{importError}</p>}
          <div className="space-y-2">
            {REPORT_DOCK_GROUPS.map((group) => (
              <details
                key={group.id}
                open={groupExpanded[group.id]}
                className="rounded border border-slate-800 bg-slate-950/30"
                data-testid={`dock-group-${group.id}`}
                onToggle={(e) => {
                  const open = (e.target as HTMLDetailsElement).open;
                  setGroupExpanded((prev) => ({ ...prev, [group.id]: open }));
                }}
              >
                <summary className="cursor-pointer px-2 py-1 text-xs font-medium text-slate-300">
                  {group.label}
                  <span className="ml-2">
                    <ReviewStepCompletionBadge state={stepCompletion[group.reviewStep]} />
                  </span>
                </summary>
                <div className="space-y-2 border-t border-slate-800/80 p-2">
                  {group.slotIds.map((slotId) => {
                    const slot = slotById.get(slotId);
                    if (!slot) return null;
                    const ok = slotPresent[slot.id];
                    const preview = previews[slot.id];
                    const stepId = slotIdToReviewStep(slot.id);
                    const slotCompletion = stepId
                      ? resolveStepCompletion(stepId, completionOptions)
                      : ok
                        ? "imported"
                        : "missing";
                    return (
                      <div
                        key={slot.id}
                        className={
                          ok
                            ? "rounded border border-dashed border-emerald-800/50 bg-emerald-950/20 p-2"
                            : "rounded border border-dashed border-slate-700 p-2"
                        }
                      >
                        <div className="flex flex-wrap items-center gap-2">
                          <p className="text-xs text-slate-300">{slot.label}</p>
                          <CompareStatusChip
                            status={ok ? "aligned" : "missing"}
                            title={ok ? "slot present" : "slot missing"}
                          />
                          <ReviewStepCompletionBadge state={slotCompletion} />
                        </div>
                        <p className="font-mono text-[10px] text-slate-500">{slot.schema}</p>
                        <div className="mt-2 flex flex-wrap gap-1">
                          <button
                            type="button"
                            className="rounded border border-slate-600 px-1.5 py-0.5 text-[10px] text-slate-300"
                            onClick={() => importSlot(slot.id)}
                          >
                            Import JSON
                          </button>
                          <button
                            type="button"
                            className="rounded border border-slate-600 px-1.5 py-0.5 text-[10px] text-slate-300 disabled:opacity-40"
                            disabled={!ok}
                            onClick={() => {
                              const text = onExportSlot(slot.id);
                              if (text) {
                                setPreviewSlot(slot.id);
                              }
                            }}
                          >
                            Preview export
                          </button>
                        </div>
                        {(previewSlot === slot.id || preview) && preview && (
                          <pre className="mt-2 max-h-24 overflow-auto text-[9px] text-slate-500">
                            {truncateJson(preview)}
                          </pre>
                        )}
                      </div>
                    );
                  })}
                </div>
              </details>
            ))}
          </div>
        </>
      )}
      {activeTab === "packet" && (
        <div className="space-y-2">
          <p className="text-[10px] text-amber-200/80">{REVIEW_PACKET_GOVERNANCE_BANNER}</p>
          <ReviewPacketGroupedSummary packet={packetPreview} />
          <div className="space-y-2" data-testid="review-packet-sections-preview">
            <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
              Packet sections (UI preview — not in export JSON)
            </p>
            {groupedSections.map(({ group, sections }) => (
              <details
                key={group.id}
                open
                className="rounded border border-slate-800 bg-slate-950/30"
                data-testid={`packet-section-group-${group.id}`}
              >
                <summary className="cursor-pointer px-2 py-1 text-[10px] font-medium text-slate-400">
                  {group.label}
                </summary>
                <div className="space-y-2 border-t border-slate-800/80 p-2">
                  {sections.map((section) => (
                    <ReviewPacketSectionCard
                      key={section.section_id}
                      section={section}
                      completionHint={section.completion_hint}
                      organizationHint={sectionOrganizationHint(section.section_id)}
                    />
                  ))}
                </div>
              </details>
            ))}
          </div>
          <pre
            className="max-h-40 overflow-auto rounded border border-slate-800 bg-slate-950 p-2 text-[9px] text-slate-400"
            data-testid="review-packet-preview"
          >
            {truncateJson(packetJson)}
          </pre>
          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={() => void handleCopyPacket()}
              title="Advisory export only — same JSON shape as download; no sections array"
            >
              Copy packet JSON (advisory only)
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={handleDownloadPacket}
              data-testid="review-packet-download"
              title="Advisory export only — same JSON shape as copy; no sections array"
            >
              Download packet JSON (advisory only)
            </button>
          </div>
          {copyStatus && (
            <p className="text-[10px] text-slate-500" data-testid="review-packet-copy-status">
              {copyStatus}
            </p>
          )}
        </div>
      )}
    </div>
  );
}
