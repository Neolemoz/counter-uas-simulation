import { useEffect, useState } from "react";
import { loadValidationMirror } from "./loadOrchestration";
import type { ValidationMirror } from "./orchestrationSchema";
import type { ExperimentNavHooks } from "@/navigation/experimentNavigation";
import { navigateToScenarioPack } from "@/navigation/experimentNavigation";
import { loadAuthoringManifest, isFingerprintStale } from "@/authoring/loadAuthoring";

type Props = {
  scenarioPackId: string | null;
  hooks?: ExperimentNavHooks;
};

export function ValidationStatusPanel({ scenarioPackId, hooks }: Props) {
  const [mirror, setMirror] = useState<ValidationMirror | null>(null);
  const [staleHint, setStaleHint] = useState(false);

  useEffect(() => {
    if (!scenarioPackId) {
      setMirror(null);
      setStaleHint(false);
      return;
    }
    let cancelled = false;
    void (async () => {
      const [m, auth] = await Promise.all([
        loadValidationMirror(scenarioPackId).catch(() => null),
        loadAuthoringManifest(scenarioPackId).catch(() => null),
      ]);
      if (cancelled) return;
      setMirror(m);
      setStaleHint(auth ? isFingerprintStale(auth, m?.checked_at) : false);
    })();
    return () => {
      cancelled = true;
    };
  }, [scenarioPackId]);

  if (!scenarioPackId) {
    return (
      <p className="text-xs text-slate-500">Select a scenario pack to view validation mirror.</p>
    );
  }

  if (!mirror) {
    return (
      <p className="text-xs text-slate-500">
        No validation mirror for <span className="font-mono">{scenarioPackId}</span>. Run
        experiment queue with validation_mirror step.
      </p>
    );
  }

  return (
    <div className="space-y-2 text-xs">
      <p className="text-slate-500">
        Pack: <span className="font-mono text-slate-300">{mirror.scenario_pack_id}</span>
      </p>
      <p className={mirror.ok ? "text-emerald-400" : "text-red-400"}>
        {mirror.ok ? "Validation mirror: pass" : "Validation mirror: issues present"}
      </p>
      {(mirror.issues ?? []).length > 0 && (
        <ul className="list-inside list-disc text-red-300/90">
          {mirror.issues!.map((issue) => (
            <li key={issue}>{issue}</li>
          ))}
        </ul>
      )}
      {(mirror.warnings ?? []).length > 0 && (
        <ul className="list-inside list-disc text-amber-200/80">
          {mirror.warnings!.map((w) => (
            <li key={w}>{w}</li>
          ))}
        </ul>
      )}
      {mirror.checked_at && (
        <p className="text-[10px] text-slate-600">Checked: {mirror.checked_at}</p>
      )}
      {staleHint && (
        <p className="text-amber-300/80">
          Staleness hint: manifest updated after validation mirror — re-run CLI record-validation
          (display only).
        </p>
      )}
      {hooks && (
        <button
          type="button"
          className="w-full rounded bg-slate-700 py-1 text-xs text-slate-200 hover:bg-slate-600"
          onClick={() => void navigateToScenarioPack(scenarioPackId, hooks)}
        >
          Open validated pack in replay
        </button>
      )}
    </div>
  );
}
