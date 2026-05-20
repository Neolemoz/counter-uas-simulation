import type { ReplaySaBundle } from "@/replay/bundleSchema";
import { ReplayModeBadge } from "./ReplayModeBadge";

type Props = {
  bundle: ReplaySaBundle | null;
  compareMode?: boolean;
  presentationMode?: boolean;
  compareTitles?: (string | undefined)[];
};

export function GovernanceChrome({ bundle, compareMode, presentationMode, compareTitles }: Props) {
  const scanGuide =
    bundle?.comprehension.scan_guide?.[0] ??
    bundle?.governance.notice ??
    "Derived replay visualization — not authoritative state.";

  return (
    <header className="border-b border-slate-700 bg-slate-900/95 px-4 py-3">
      <div className="mb-2 flex flex-wrap items-center gap-3">
        <ReplayModeBadge />
        {compareMode && (
          <span className="rounded border border-amber-700/60 bg-amber-950/50 px-2 py-0.5 text-xs font-medium text-amber-200">
            COMPARE — explanatory diff only
          </span>
        )}
        {presentationMode && (
          <span className="rounded border border-violet-700/60 bg-violet-950/50 px-2 py-0.5 text-xs font-medium text-violet-200">
            PRESENTATION — guided replay review
          </span>
        )}
        <h1 className="text-lg font-semibold text-slate-100">SA-R0 Replay Viewer</h1>
        {compareMode && compareTitles ? (
          <span className="text-sm text-slate-400">
            A: {compareTitles[0] ?? "—"} | B: {compareTitles[1] ?? "—"}
          </span>
        ) : (
          bundle?.scenario.title && (
            <span className="text-sm text-slate-400">{bundle.scenario.title}</span>
          )
        )}
        {bundle?.scenario.ingress_archetype && !compareMode && (
          <span className="rounded bg-slate-800 px-2 py-0.5 text-xs text-slate-500">
            {bundle.scenario.ingress_archetype} ingress
          </span>
        )}
      </div>
      <p className="text-sm text-amber-100/90">
        {compareMode
          ? "Side-by-side replay comparison for topology and narrative interpretation — not operational benchmarking."
          : presentationMode
            ? "Guided presentation walkthrough — derived replay summaries only, not operational monitoring."
            : scanGuide}
      </p>
      <p className="mt-1 text-xs text-slate-500">
        {bundle?.georef_display.caveat ??
          "Scenario-local ENU with fictional georef — not deployed geography."}
      </p>
    </header>
  );
}
