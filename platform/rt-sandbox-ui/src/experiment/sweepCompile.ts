import {
  experimentBatchSpecSchema,
  experimentSweepCatalogSchema,
  type ExperimentBatchSpec,
  type SweepGroup,
} from "./experimentSchema";

export function parseSweepCatalog(data: unknown) {
  return experimentSweepCatalogSchema.parse(data);
}

export function compileSweepGroup(
  group: SweepGroup,
  experimentId: string,
): ExperimentBatchSpec {
  if (group.compile_strategy !== "explicit_list") {
    throw new Error(`unsupported compile_strategy: ${group.compile_strategy}`);
  }
  const defaultDwell = group.default_dwell_s ?? 2;
  const runs = group.sweep_entries.map((entry) => ({
    run_id: entry.run_id_suffix ?? entry.entry_id,
    label: entry.label,
    template_id: entry.template_id,
    dwell_s: entry.dwell_s ?? defaultDwell,
    tactical_mode_hint: entry.tactical_mode_hint,
  }));
  return experimentBatchSpecSchema.parse({
    schema: "rt_experiment_batch_v1",
    experiment_id: experimentId,
    default_dwell_s: defaultDwell,
    runs,
  });
}

export function batchSpecToYaml(spec: ExperimentBatchSpec): string {
  const lines: string[] = [
    `schema: ${spec.schema}`,
    `experiment_id: ${spec.experiment_id}`,
  ];
  if (spec.default_dwell_s != null) {
    lines.push(`default_dwell_s: ${spec.default_dwell_s}`);
  }
  lines.push("runs:");
  for (const run of spec.runs) {
    lines.push(`  - run_id: ${run.run_id}`);
    lines.push(`    label: ${yamlQuote(run.label)}`);
    if (run.dwell_s != null) lines.push(`    dwell_s: ${run.dwell_s}`);
    if (run.template_id) lines.push(`    template_id: ${run.template_id}`);
    if (run.tactical_mode_hint) {
      lines.push(`    tactical_mode_hint: ${run.tactical_mode_hint}`);
    }
    if (run.experiment_class) lines.push(`    experiment_class: ${run.experiment_class}`);
    if (run.spec_fingerprint) lines.push(`    spec_fingerprint: ${run.spec_fingerprint}`);
    if (run.repeat_index != null) lines.push(`    repeat_index: ${run.repeat_index}`);
    if (run.repeat_group_id) lines.push(`    repeat_group_id: ${run.repeat_group_id}`);
    if (run.terrain_profile_ref) {
      lines.push(`    terrain_profile_ref: ${run.terrain_profile_ref}`);
    }
    if (run.f4_layer_preset) lines.push(`    f4_layer_preset: ${run.f4_layer_preset}`);
  }
  return `${lines.join("\n")}\n`;
}

function yamlQuote(s: string): string {
  if (/^[a-zA-Z0-9 _-]+$/.test(s)) return s;
  return JSON.stringify(s);
}

export function suggestedExperimentId(groupId: string): string {
  const date = new Date().toISOString().slice(0, 10);
  return `sweep-${groupId}-${date}`;
}
