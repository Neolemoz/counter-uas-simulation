import { useEffect, useMemo, useState } from "react";
import { useClockStore } from "@/replay/clockStore";
import { useCompareStore } from "@/replay/compareStore";
import { loadScenarioCatalog } from "@/replay/loadCatalog";
import type { CatalogPack } from "@/replay/catalogSchema";
import { loadOrchestrationQueue, readOrchestrationQueueFromUrl } from "@/orchestration/loadOrchestration";
import { loadValidationMirror } from "@/orchestration/loadOrchestration";
import { loadAuthoringManifest } from "@/authoring/loadAuthoring";
import type { ExperimentRunQueue } from "@/orchestration/orchestrationSchema";
import { useCorpusStore } from "@/replay/corpus/useCorpusStore";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import {
  navigateFromQueueJob,
  navigateToComparePair,
  navigateToScenarioPack,
  navigateToCorpusEntryById,
} from "@/navigation/experimentNavigation";
import { sandboxAccents, sandboxTypography } from "@/theme/sandboxTheme";

type LineageHop = {
  label: string;
  detail: string;
  action?: () => void;
  muted?: boolean;
};

type Props = {
  scenarioPackId?: string | null;
  hooks: ExperimentNavHooks;
};

export function ExperimentLineagePanel({ scenarioPackId, hooks }: Props) {
  const bundle = useClockStore((s) => s.bundle);
  const compareMode = useCompareStore((s) => s.mode === "compare");
  const activePairId = useCompareStore((s) => s.activePairId);
  const corpusEntryId = useCorpusStore((s) => s.selectedEntryId);
  const [pack, setPack] = useState<CatalogPack | null>(null);
  const [mirror, setMirror] = useState<Awaited<ReturnType<typeof loadValidationMirror>> | null>(null);
  const [queue, setQueue] = useState<ExperimentRunQueue | null>(null);
  const [authoringStatus, setAuthoringStatus] = useState<string | null>(null);

  const packId =
    scenarioPackId ?? bundle?.scenario.catalog_pack_id ?? bundle?.scenario.scenario_id ?? null;

  useEffect(() => {
    if (!packId) {
      setPack(null);
      setMirror(null);
      return;
    }
    void loadScenarioCatalog()
      .then((c) => setPack(c.packs.find((p) => p.pack_id === packId) ?? null))
      .catch(() => setPack(null));
    void loadValidationMirror(packId).then(setMirror).catch(() => setMirror(null));
    void loadAuthoringManifest(packId)
      .then((m) => setAuthoringStatus(m?.promotion_status ?? null))
      .catch(() => setAuthoringStatus(null));
  }, [packId]);

  useEffect(() => {
    const qid = readOrchestrationQueueFromUrl();
    if (!qid) {
      setQueue(null);
      return;
    }
    void loadOrchestrationQueue(qid).then(setQueue).catch(() => setQueue(null));
  }, [packId]);

  const matchingJob = useMemo(() => {
    if (!queue || !packId) return null;
    return queue.jobs.find((j) => j.scenario_pack_id === packId) ?? queue.jobs[0] ?? null;
  }, [queue, packId]);

  const hops: LineageHop[] = useMemo(() => {
    const list: LineageHop[] = [];
    if (packId) {
      list.push({
        label: "Scenario pack",
        detail: pack?.title ?? packId,
        action: () => void navigateToScenarioPack(packId, { ...hooks, segment: "replay" }),
      });
    }
    if (authoringStatus) {
      list.push({
        label: "Authoring promotion",
        detail: authoringStatus,
        muted: authoringStatus === "draft" || authoringStatus === "linted",
      });
    }
    if (mirror) {
      list.push({
        label: "Validation mirror",
        detail: mirror.ok ? "pass (offline lint)" : "issues present",
        muted: !mirror.ok,
      });
    }
    if (matchingJob) {
      list.push({
        label: "Queue job",
        detail: `${matchingJob.job_id} · ${matchingJob.status}`,
        action: () => void navigateFromQueueJob(matchingJob, hooks),
      });
    }
    if (bundle) {
      list.push({
        label: "Replay bundle",
        detail: bundle.scenario.title,
        action: packId
          ? () => void navigateToScenarioPack(packId, { ...hooks, segment: "replay" })
          : undefined,
      });
    }
    if (corpusEntryId) {
      list.push({
        label: "Corpus entry",
        detail: corpusEntryId,
        action: () => void navigateToCorpusEntryById(corpusEntryId, hooks),
      });
    }
    if (compareMode) {
      list.push({
        label: "Compare session",
        detail: activePairId ?? "custom A/B",
      });
    }
    const pairIds = pack?.compare_pair_ids ?? [];
    if (pairIds.length > 0 && !compareMode) {
      list.push({
        label: "Compare pairs",
        detail: pairIds.join(", "),
        action: () => void navigateToComparePair(pairIds[0]!, { ...hooks, segment: "compare" }),
      });
    }
    return list;
  }, [
    packId,
    pack,
    authoringStatus,
    mirror,
    matchingJob,
    bundle,
    corpusEntryId,
    compareMode,
    activePairId,
    hooks,
  ]);

  if (hops.length === 0) {
    return (
      <p className={sandboxTypography.caption}>
        Select a scenario pack or load a replay bundle to see experiment lineage.
      </p>
    );
  }

  return (
    <div className="space-y-2">
      <p className={sandboxTypography.caption}>
        Experiment lineage — explanatory mirrors only, not operational state.
      </p>
      <ol className="space-y-1 border-l border-slate-700/60 pl-3">
        {hops.map((hop) => (
          <li key={`${hop.label}-${hop.detail}`} className="relative">
            <span className="absolute -left-[7px] top-1.5 h-2 w-2 rounded-full bg-slate-600" />
            {hop.action ? (
              <button
                type="button"
                className={`text-left ${sandboxAccents.link}`}
                onClick={hop.action}
              >
                <span className="font-medium text-slate-300">{hop.label}</span>
                <span className={`block ${sandboxTypography.caption}`}>{hop.detail}</span>
              </button>
            ) : (
              <div className={hop.muted ? "text-amber-200/80" : "text-slate-400"}>
                <span className="font-medium text-slate-300">{hop.label}</span>
                <span className={`block ${sandboxTypography.caption}`}>{hop.detail}</span>
              </div>
            )}
          </li>
        ))}
      </ol>
    </div>
  );
}
