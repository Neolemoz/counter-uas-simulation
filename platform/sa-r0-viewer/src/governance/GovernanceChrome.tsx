import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { WorkspaceSegment } from "@/workspace/types";
import { segmentBanner } from "@/workspace/segmentBanners";
import { ReplayModeBadge } from "./ReplayModeBadge";
import { sandboxTypography } from "@/theme/sandboxTheme";

type Props = {
  bundle: ReplaySaBundle | null;
  compareMode?: boolean;
  presentationMode?: boolean;
  workspaceSegment?: WorkspaceSegment;
  compareTitles?: (string | undefined)[];
  scenarioPackId?: string | null;
  orchestrationQueueId?: string | null;
  validationOk?: boolean | null;
  authoringMirror?: boolean;
};

export function GovernanceChrome({
  bundle,
  compareMode,
  presentationMode,
  workspaceSegment = "replay",
  compareTitles,
  scenarioPackId,
  orchestrationQueueId,
  validationOk,
  authoringMirror = false,
}: Props) {
  const scanGuide =
    bundle?.comprehension.scan_guide?.[0] ??
    bundle?.governance.notice ??
    "Derived replay visualization — not authoritative state.";

  return (
    <header className="border-b border-slate-800/80 bg-slate-950/95 px-4 py-3">
      <div className="mb-2 flex flex-wrap items-baseline gap-x-3 gap-y-1">
        <ReplayModeBadge />
        {compareMode && (
          <span className="rounded-md border border-amber-800/35 bg-amber-950/25 px-2 py-0.5 text-[11px] font-medium text-amber-200/90">
            Compare — explanatory diff
          </span>
        )}
        {presentationMode && (
          <span className="rounded-md border border-violet-800/35 bg-violet-950/25 px-2 py-0.5 text-[11px] font-medium text-violet-200/90">
            Presentation — guided review
          </span>
        )}
        <h1 className={sandboxTypography.pageTitle}>Replay Experimentation Sandbox</h1>
      </div>
      {compareMode && compareTitles ? (
        <p className="mb-1 text-sm text-slate-400">
          <span className="text-slate-500">A</span> {compareTitles[0] ?? "—"}
          <span className="mx-2 text-slate-600">|</span>
          <span className="text-slate-500">B</span> {compareTitles[1] ?? "—"}
        </p>
      ) : (
        bundle?.scenario.title && (
          <p className="mb-1 text-sm text-slate-300">{bundle.scenario.title}</p>
        )
      )}
      {(scenarioPackId || orchestrationQueueId || validationOk != null) && (
        <p className={`mb-1 ${sandboxTypography.caption}`}>
          Experiment context (mirror):{" "}
          {scenarioPackId && <span className="font-mono text-slate-400">pack={scenarioPackId}</span>}
          {orchestrationQueueId && (
            <span className="ml-2 font-mono text-slate-400">queue={orchestrationQueueId}</span>
          )}
          {validationOk != null && (
            <span className={`ml-2 ${validationOk ? "text-emerald-400/80" : "text-amber-300/80"}`}>
              validation={validationOk ? "pass" : "issues"}
            </span>
          )}
        </p>
      )}
      <p className="text-sm leading-relaxed text-amber-100/85">
        {compareMode
          ? "Side-by-side replay comparison for topology and narrative interpretation — not operational benchmarking."
          : presentationMode
            ? "Guided presentation walkthrough — derived replay summaries only, not operational monitoring."
            : scanGuide}
      </p>
      <p className={`mt-1.5 ${sandboxTypography.caption}`}>
        {segmentBanner(workspaceSegment, { authoringMirror })}
      </p>
      <p className={`mt-1 ${sandboxTypography.caption}`}>
        {bundle?.georef_display.caveat ??
          "Scenario-local ENU with fictional georef — not deployed geography."}
      </p>
    </header>
  );
}
