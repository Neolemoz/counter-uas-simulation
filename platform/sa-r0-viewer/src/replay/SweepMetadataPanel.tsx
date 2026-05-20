import { useSweepStore } from "./useSweepStore";
import { SWEEP_KIND_LABELS } from "./sweepSchema";

export function SweepMetadataPanel() {
  const sweep = useSweepStore((s) => s.sweep);
  const memberIndex = useSweepStore((s) => s.memberIndex);
  if (!sweep) return null;

  const member = sweep.members[memberIndex];
  const lineage = sweep.lineage as Record<string, unknown> | undefined;
  const linkage = sweep.topology_linkage as Record<string, unknown> | undefined;

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Sweep metadata</h2>
      <dl className="space-y-1 text-xs text-slate-400">
        <div>
          <dt className="inline font-medium text-slate-500">Sweep: </dt>
          <dd className="inline text-slate-300">{sweep.title}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">Kind: </dt>
          <dd className="inline text-slate-300">
            {SWEEP_KIND_LABELS[sweep.sweep_kind] ?? sweep.sweep_kind}
          </dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">Members: </dt>
          <dd className="inline text-slate-300">{sweep.members.length}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">Baseline: </dt>
          <dd className="inline text-slate-300">{sweep.baseline_topology_key}</dd>
        </div>
        {member && (
          <div>
            <dt className="inline font-medium text-slate-500">Active: </dt>
            <dd className="inline text-slate-300">
              {member.member_id} / {member.pack_id}
              {member.seed != null ? ` (seed ${member.seed})` : ""}
            </dd>
          </div>
        )}
        {lineage?.seed_base != null && (
          <div>
            <dt className="inline font-medium text-slate-500">Seed base: </dt>
            <dd className="inline text-slate-300">{String(lineage.seed_base)}</dd>
          </div>
        )}
        {Array.isArray(linkage?.topology_keys) && (
          <div>
            <dt className="font-medium text-slate-500">Topology keys</dt>
            <dd className="text-slate-300">{(linkage.topology_keys as string[]).join(", ")}</dd>
          </div>
        )}
      </dl>
      <p className="mt-2 text-[10px] text-slate-500">{sweep.governance.notice}</p>
    </section>
  );
}
