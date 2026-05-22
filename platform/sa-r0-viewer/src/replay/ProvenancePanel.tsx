import type { ReplaySaBundle } from "./bundleSchema";

type Props = { bundle: ReplaySaBundle };

function Row({ label, value }: { label: string; value: string }) {
  if (!value || value === "—") return null;
  return (
    <div>
      <dt className="inline font-medium text-slate-500">{label}: </dt>
      <dd className="inline break-all font-mono text-xs">{value}</dd>
    </div>
  );
}

export function ProvenancePanel({ bundle }: Props) {
  const lineage = bundle.lineage as Record<string, unknown>;
  const src = bundle.source_artifacts ?? {};
  const packMeta = (src as { scenario_pack_metadata?: { provenance?: Record<string, string> } })
    .scenario_pack_metadata;
  const provenance = bundle.scenario.provenance ?? packMeta?.provenance;

  return (
    <div className="border-t border-slate-700 pt-2">
      <h3 className="mb-2 text-xs font-semibold uppercase text-slate-500">Provenance (explanatory)</h3>
      <dl className="grid gap-1 text-slate-400">
        <Row label="run_id" value={String(lineage.run_id ?? "—")} />
        <Row label="cohort" value={String(lineage.cohort ?? "—")} />
        <Row label="log_path" value={String(lineage.log_path ?? "—")} />
        <Row label="meta_path" value={String(lineage.meta_path ?? "—")} />
        <Row label="launch_command" value={String(lineage.launch_command ?? "—")} />
        <Row label="created_utc" value={String(lineage.created_utc ?? "—")} />
        <Row label="git_commit" value={String(lineage.git_commit ?? "—")} />
        <Row
          label="git_dirty"
          value={lineage.git_dirty === undefined ? "—" : String(lineage.git_dirty)}
        />
        <Row label="evaluation_row" value={String(lineage.evaluation_row_path ?? "—")} />
        <Row label="scenario_pack" value={String(src.scenario_pack ?? "—")} />
        <Row label="seed" value={String(lineage.seed ?? "—")} />
      </dl>
      {provenance?.fictional_disclaimer && (
        <p className="mt-2 text-xs text-amber-200/90">{provenance.fictional_disclaimer}</p>
      )}
      {provenance?.fixture_source && (
        <p className="mt-1 text-[10px] text-slate-600">Fixture: {provenance.fixture_source}</p>
      )}
      {(bundle.scenario.overlay_descriptors?.length ?? 0) > 0 && (
        <div className="mt-2">
          <p className="text-[10px] font-medium uppercase text-slate-500">Pack overlays</p>
          <ul className="text-xs text-slate-500">
            {bundle.scenario.overlay_descriptors!.map((d) => (
              <li key={d.kind}>
                {d.kind}
                {d.count != null ? ` ×${d.count}` : ""}
              </li>
            ))}
          </ul>
          <p className="mt-1 text-[10px] text-slate-600">
            LOS segments are derived at pack time — replay-local proxies, not sensor physics.
          </p>
        </div>
      )}
      {bundle.scenario.terrain_model?.caveat && (
        <p className="mt-2 text-xs text-amber-200/80">{bundle.scenario.terrain_model.caveat}</p>
      )}
      <p className="mt-2 text-[10px] text-slate-600">
        {bundle.governance.notice} Not deployed geography or operational effectiveness.
      </p>
    </div>
  );
}
