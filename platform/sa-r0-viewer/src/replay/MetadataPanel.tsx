import { ProvenancePanel } from "./ProvenancePanel";
import type { ReplaySaBundle } from "./bundleSchema";

type Props = { bundle: ReplaySaBundle };

export function MetadataPanel({ bundle }: Props) {
  const launch = bundle.scenario.launch_geometry ?? {};

  return (
    <section className="rounded border border-slate-700 bg-slate-900/80 p-3 text-sm">
      <h2 className="mb-2 font-semibold text-slate-200">Replay metadata</h2>
      <dl className="mb-3 grid gap-1 text-slate-400">
        <div>
          <dt className="inline font-medium text-slate-500">scenario: </dt>
          <dd className="inline">{bundle.scenario.scenario_id}</dd>
        </div>
        <div>
          <dt className="inline font-medium text-slate-500">title: </dt>
          <dd className="inline">{bundle.scenario.title}</dd>
        </div>
        {bundle.scenario.catalog_pack_id && (
          <div>
            <dt className="inline font-medium text-slate-500">catalog pack: </dt>
            <dd className="inline">{bundle.scenario.catalog_pack_id}</dd>
          </div>
        )}
        {bundle.scenario.ingress_archetype && (
          <div>
            <dt className="inline font-medium text-slate-500">ingress: </dt>
            <dd className="inline">{bundle.scenario.ingress_archetype}</dd>
          </div>
        )}
        <div>
          <dt className="inline font-medium text-slate-500">topology tags: </dt>
          <dd className="inline">{(bundle.scenario.topology_tags ?? []).join(", ") || "—"}</dd>
        </div>
        {(bundle.scenario.replay_tags?.length ?? 0) > 0 && (
          <div>
            <dt className="inline font-medium text-slate-500">replay tags: </dt>
            <dd className="inline">{bundle.scenario.replay_tags!.join(", ")}</dd>
          </div>
        )}
        {bundle.scenario.replay_duration_class && (
          <div>
            <dt className="inline font-medium text-slate-500">duration class: </dt>
            <dd className="inline">{bundle.scenario.replay_duration_class}</dd>
          </div>
        )}
        {bundle.scenario.terrain_profile && (
          <div>
            <dt className="inline font-medium text-slate-500">terrain profile: </dt>
            <dd className="inline">{bundle.scenario.terrain_profile}</dd>
          </div>
        )}
        {bundle.comparison_hints?.sensor_layout_id && (
          <div>
            <dt className="inline font-medium text-slate-500">sensor layout: </dt>
            <dd className="inline font-mono text-[10px]">
              {bundle.comparison_hints.sensor_layout_id}
            </dd>
          </div>
        )}
        {bundle.comparison_hints?.compare_mode && (
          <div>
            <dt className="inline font-medium text-slate-500">compare mode: </dt>
            <dd className="inline">{bundle.comparison_hints.compare_mode}</dd>
          </div>
        )}
        {Object.keys(launch).length > 0 && (
          <div>
            <dt className="font-medium text-slate-500">launch_geometry</dt>
            <dd className="font-mono text-xs">{JSON.stringify(launch)}</dd>
          </div>
        )}
      </dl>
      <ProvenancePanel bundle={bundle} />
    </section>
  );
}
