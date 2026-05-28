import { REPORT_DOCK_SLOTS } from "./experimentUnifiedReview";

export type ReportDockPresence = {
  f1_analytics: boolean;
  f3_annex: boolean;
  f5_metrics: boolean;
  f5b_fidelity: boolean;
};

export function ExperimentReportDockShell({ presence }: { presence: ReportDockPresence }) {
  const slotPresent: Record<string, boolean> = {
    f1_analytics: presence.f1_analytics,
    f3_annex: presence.f3_annex,
    f5_metrics: presence.f5_metrics,
    f5b_fidelity: presence.f5b_fidelity,
  };

  return (
    <div
      className="space-y-2 rounded border border-slate-800 bg-slate-950/40 p-2"
      data-testid="report-dock-shell"
    >
      <p className="text-[10px] font-semibold uppercase tracking-wide text-slate-500">
        Report dock (read-only)
      </p>
      <div className="grid gap-2 sm:grid-cols-2">
        {REPORT_DOCK_SLOTS.map((slot) => {
          const ok = slotPresent[slot.id];
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
                {ok ? "present (from workbench state)" : "missing — import or derive below"}
              </p>
            </div>
          );
        })}
      </div>
    </div>
  );
}
