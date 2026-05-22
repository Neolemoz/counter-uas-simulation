import type { CatalogPack } from "@/replay/catalogSchema";
import type { ReplaySaBundle } from "@/replay/bundleSchema";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { navigateToComparePair, navigateToScenarioPack } from "@/navigation/experimentNavigation";
import { usePresentationStore } from "@/replay/presentation/presentationStore";
import { useWorkspaceSegmentStore } from "@/workspace/workspaceSegmentStore";

type Props = {
  pack: CatalogPack;
  bundle: ReplaySaBundle | null;
  hooks: ExperimentNavHooks;
};

export function ScenarioLoadedSummaryCard({ pack, bundle, hooks }: Props) {
  const setSegment = useWorkspaceSegmentStore((s) => s.setUserSegment);
  const enterWalkthrough = usePresentationStore((s) => s.enterBundleWalkthrough);

  return (
    <div className="rounded border border-cyan-900/40 bg-cyan-950/20 p-3 text-sm">
      <h2 className="mb-1 font-semibold text-cyan-100">{pack.title}</h2>
      <p className="mb-2 text-xs text-slate-400">
        {bundle
          ? "Replay bundle loaded — open Replay segment for spatial review."
          : "Select a pack to load its demo replay bundle."}
      </p>
      <div className="flex flex-wrap gap-2">
        <button
          type="button"
          className="rounded bg-slate-700 px-2 py-1 text-xs text-slate-200 hover:bg-slate-600"
          onClick={() => {
            setSegment("replay");
            if (!bundle) void navigateToScenarioPack(pack.pack_id, hooks);
          }}
        >
          Open replay review
        </button>
        {(pack.compare_pair_ids ?? []).slice(0, 2).map((pairId) => (
          <button
            key={pairId}
            type="button"
            className="rounded bg-amber-900/40 px-2 py-1 text-xs text-amber-100 hover:bg-amber-900/60"
            onClick={() => void navigateToComparePair(pairId, hooks)}
          >
            Compare: {pairId}
          </button>
        ))}
        {bundle?.presentation?.chapters?.length ? (
          <button
            type="button"
            className="rounded bg-violet-900/40 px-2 py-1 text-xs text-violet-100 hover:bg-violet-900/60"
            onClick={() => {
              enterWalkthrough(0);
              setSegment("report");
            }}
          >
            Bundle walkthrough
          </button>
        ) : null}
      </div>
    </div>
  );
}
