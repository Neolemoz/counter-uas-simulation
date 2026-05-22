import { useEffect, useState } from "react";
import { loadAuthoringManifest } from "./loadAuthoring";
import { loadOrchestrationIndex } from "@/orchestration/loadOrchestration";
import type { ScenarioAuthoringManifest } from "./authoringSchema";

type Props = {
  packId: string | null;
};

export function AuthoringOrchestrationHandoffPanel({ packId }: Props) {
  const [manifest, setManifest] = useState<ScenarioAuthoringManifest | null>(null);
  const [orchIndex, setOrchIndex] = useState<{ id: string; kind: string }[]>([]);

  useEffect(() => {
    if (!packId) {
      setManifest(null);
      return;
    }
    loadAuthoringManifest(packId)
      .then(setManifest)
      .catch(() => setManifest(null));
  }, [packId]);

  useEffect(() => {
    loadOrchestrationIndex()
      .then((entries) => setOrchIndex(entries.map((e) => ({ id: e.id, kind: e.kind }))))
      .catch(() => setOrchIndex([]));
  }, []);

  if (!packId) {
    return <p className="text-xs text-slate-500">Select a scenario pack.</p>;
  }

  const refs = manifest?.orchestration_handoff_refs ?? [];
  const ready =
    manifest?.promotion_status === "promoted" ||
    manifest?.promotion_status === "orchestration_ready";

  const resolveLabel = (id: string | undefined, kind: string) => {
    if (!id) return null;
    const hit = orchIndex.find((e) => e.id === id && e.kind === kind);
    return hit ? `${id} (${kind})` : id;
  };

  return (
    <div className="space-y-2 text-xs">
      <p className="text-slate-500">Orchestration handoff (references only — no launch)</p>
      <p className={ready ? "text-emerald-400/90" : "text-amber-300/80"}>
        {manifest?.promotion_status === "orchestration_ready"
          ? "Orchestration ready — handoff refs recorded (CLI runner only)."
          : ready
            ? "Pack promotion allows orchestration manifest reference (CLI runner)."
            : "Promote pack via CLI before job manifest handoff."}
      </p>
      {manifest?.validation_snapshot_ref && (
        <p className="break-all font-mono text-[10px] text-slate-500">
          validation: {manifest.validation_snapshot_ref}
        </p>
      )}
      {refs.length > 0 ? (
        <ul className="list-inside list-disc text-slate-400">
          {refs.map((r, i) => (
            <li key={`${r.manifest_id}-${i}`}>
              {resolveLabel(r.manifest_id, "manifests") ?? r.manifest_id ?? "manifest"}
              {r.job_id ? ` / job ${r.job_id}` : ""}
              {r.queue_mirror_id ? (
                <>
                  {" "}
                  → {resolveLabel(r.queue_mirror_id, "queues") ?? r.queue_mirror_id}
                </>
              ) : null}
            </li>
          ))}
        </ul>
      ) : (
        <p className="text-slate-500">
          No handoff refs on manifest. Example manifest:{" "}
          <span className="font-mono">fixtures/orchestration/manifests/</span>
        </p>
      )}
    </div>
  );
}
