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
}: {
  v2State: WorkbenchV2State;
  manifest: ExperimentManifest;
  presence: ReportDockPresence;
  previews: Partial<Record<string, string>>;
  onImportSlot: (slotId: string, text: string) => string | null;
  onExportSlot: (slotId: string) => string | null;
  packetTabFocusToken?: number;
}) {
  const [activeTab, setActiveTab] = useState<"slots" | "packet">("slots");
  const [importError, setImportError] = useState<string | null>(null);
  const [previewSlot, setPreviewSlot] = useState<string | null>(null);
  const [copyStatus, setCopyStatus] = useState<string | null>(null);

  useEffect(() => {
    if (packetTabFocusToken != null && packetTabFocusToken > 0) {
      setActiveTab("packet");
    }
  }, [packetTabFocusToken]);

  const packetPreview = useMemo(
    () => buildReviewPacketPreview({ v2State, manifest, presence }),
    [v2State, manifest, presence],
  );

  const packetJson = useMemo(
    () => JSON.stringify(packetPreview, null, 2),
    [packetPreview],
  );

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

  const slotPresent: Record<string, boolean> = {
    f1_analytics: presence.f1_analytics,
    f3_annex: presence.f3_annex,
    f5_metrics: presence.f5_metrics,
    f5b_fidelity: presence.f5b_fidelity,
  };

  const handleCopyPacket = async () => {
    const result = await copyReviewPacketJson(packetPreview);
    if (result.ok) {
      setCopyStatus("copied");
    } else {
      setCopyStatus(result.error);
    }
  };

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
          onClick={() => setActiveTab("packet")}
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
          <div className="grid gap-2 sm:grid-cols-2">
            {REPORT_DOCK_SLOTS.map((slot) => {
              const ok = slotPresent[slot.id];
              const preview = previews[slot.id];
              return (
                <div
                  key={slot.id}
                  className={
                    ok
                      ? "rounded border border-dashed border-emerald-800/50 bg-emerald-950/20 p-2"
                      : "rounded border border-dashed border-slate-700 p-2"
                  }
                >
                  <p className="text-xs text-slate-300">{slot.label}</p>
                  <p className="font-mono text-[10px] text-slate-500">{slot.schema}</p>
                  <p className="mt-1 text-[10px] text-slate-400">
                    {ok ? "present" : "missing"}
                  </p>
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
        </>
      )}
      {activeTab === "packet" && (
        <div className="space-y-2">
          <p className="text-[10px] text-amber-200/80">{REVIEW_PACKET_GOVERNANCE_BANNER}</p>
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
            >
              Copy packet JSON (advisory only)
            </button>
            <button
              type="button"
              className="rounded border border-slate-600 bg-slate-800 px-2 py-1 text-xs text-slate-200"
              onClick={() => downloadReviewPacket(packetPreview)}
              data-testid="review-packet-download"
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
