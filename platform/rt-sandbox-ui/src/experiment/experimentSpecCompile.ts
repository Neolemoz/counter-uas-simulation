import {
  experimentBatchSpecSchema,
  experimentSpecSchema,
  type ExperimentBatchSpec,
  type ExperimentClass,
  type ExperimentSpec,
} from "./experimentSchema";
import { sha256Hex16 } from "./sha256Hex";
import { batchSpecToYaml } from "./sweepCompile";
import {
  assertAllowedMatrixAxis,
  assertBuiltinTemplateId,
  assertTemplateRefBlocked,
} from "./templateGuards";

export type F5RunMeta = {
  experiment_class: ExperimentClass;
  spec_fingerprint: string;
  matrix_coords?: Record<string, string> | null;
  repeat_index?: number;
  repeat_group_id?: string;
  terrain_profile_ref?: string;
  f4_layer_preset?: string;
};

export type CompileResult = {
  batch: ExperimentBatchSpec;
  runMeta: Map<string, F5RunMeta>;
};

export function parseExperimentSpec(data: unknown): ExperimentSpec {
  return experimentSpecSchema.parse(data);
}

function canonicalizeForFingerprint(spec: ExperimentSpec): Record<string, unknown> {
  const copy = JSON.parse(JSON.stringify(spec)) as Record<string, unknown>;
  delete copy.experiment_id;
  return sortKeysDeep(copy) as Record<string, unknown>;
}

function sortKeysDeep(value: unknown): unknown {
  if (Array.isArray(value)) {
    return value.map(sortKeysDeep);
  }
  if (value && typeof value === "object") {
    const obj = value as Record<string, unknown>;
    const sorted: Record<string, unknown> = {};
    for (const key of Object.keys(obj).sort()) {
      sorted[key] = sortKeysDeep(obj[key]);
    }
    return sorted;
  }
  return value;
}

export function computeSpecFingerprint(spec: ExperimentSpec): string {
  const canonical = JSON.stringify(canonicalizeForFingerprint(spec));
  return sha256Hex16(canonical);
}

function slugifyRunId(parts: string[]): string {
  const raw = `m-${parts.join("-")}`
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
  if (raw.length <= 64) return raw;
  const hash = sha256Hex16(raw).slice(0, 8);
  return `${raw.slice(0, 55)}-${hash}`;
}

function axisSignature(coords: Record<string, string>): string {
  return Object.keys(coords)
    .sort()
    .map((k) => `${k}=${coords[k]}`)
    .join(";");
}

function validateTemplateId(templateId: string): void {
  assertBuiltinTemplateId(templateId);
}

function validateSpecRef(ref: string | undefined): void {
  if (ref) assertTemplateRefBlocked(ref);
}

export function assertCompileableSpec(spec: ExperimentSpec): void {
  const fp = computeSpecFingerprint(spec);
  if (!fp) throw new Error("spec_fingerprint empty");

  const strategy = spec.compile_strategy;
  const cls = spec.experiment_class;

  if (cls === "parameter_matrix" && strategy !== "cartesian") {
    throw new Error("parameter_matrix requires cartesian compile_strategy");
  }
  if (cls === "repeatability_sweep" && strategy !== "repeat_expand") {
    throw new Error("repeatability_sweep requires repeat_expand compile_strategy");
  }
  if (
    (cls === "terrain_comparison" ||
      cls === "sensor_range_comparison" ||
      cls === "tactical_mode_comparison") &&
    strategy !== "explicit_list"
  ) {
    throw new Error(`${cls} requires explicit_list compile_strategy`);
  }

  if (strategy === "cartesian") {
    const axes = spec.matrix_axes ?? [];
    if (axes.length < 1) throw new Error("cartesian requires matrix_axes");
    for (const axis of axes) {
      assertAllowedMatrixAxis(axis.axis_id);
      if (!axis.values.length) throw new Error(`empty values for axis ${axis.axis_id}`);
      for (const v of axis.values) {
        const s = String(v);
        if (axis.axis_id === "template_id") validateTemplateId(s);
        else validateSpecRef(s);
      }
    }
  }

  if (strategy === "repeat_expand") {
    const rc = spec.repeat_config;
    if (!rc) throw new Error("repeat_expand requires repeat_config");
    if (rc.jitter_s !== 0) throw new Error("repeat_config.jitter_s must be 0");
    if (rc.count < 2) throw new Error("repeat_config.count must be >= 2");
    validateTemplateId(rc.base_entry.template_id);
  }

  if (strategy === "explicit_list") {
    const entries = spec.spec_entries ?? [];
    if (!entries.length) throw new Error("explicit_list requires spec_entries");
    for (const entry of entries) {
      validateTemplateId(entry.template_id);
      validateSpecRef(entry.terrain_profile_ref);
      validateSpecRef(entry.f4_layer_preset);
    }
  }
}

function buildBatchRun(
  fingerprint: string,
  experimentClass: ExperimentClass,
  run: {
    run_id: string;
    label: string;
    template_id?: string | null;
    dwell_s?: number;
    tactical_mode_hint?: string;
    meta: F5RunMeta;
  },
): ExperimentBatchSpec["runs"][number] {
  return {
    run_id: run.run_id,
    label: run.label,
    template_id: run.template_id ?? null,
    dwell_s: run.dwell_s,
    tactical_mode_hint: run.tactical_mode_hint,
    experiment_class: experimentClass,
    spec_fingerprint: fingerprint,
    matrix_coords: run.meta.matrix_coords ?? null,
    repeat_index: run.meta.repeat_index,
    repeat_group_id: run.meta.repeat_group_id,
    terrain_profile_ref: run.meta.terrain_profile_ref,
    f4_layer_preset: run.meta.f4_layer_preset,
  };
}

function compileExplicitList(spec: ExperimentSpec, fingerprint: string): CompileResult {
  const defaultDwell = spec.default_dwell_s ?? 2;
  const runMeta = new Map<string, F5RunMeta>();
  const runs: ExperimentBatchSpec["runs"] = [];

  for (const entry of spec.spec_entries ?? []) {
    const meta: F5RunMeta = {
      experiment_class: spec.experiment_class,
      spec_fingerprint: fingerprint,
      terrain_profile_ref: entry.terrain_profile_ref,
      f4_layer_preset: entry.f4_layer_preset,
    };
    runs.push(
      buildBatchRun(fingerprint, spec.experiment_class, {
        run_id: entry.entry_id,
        label: entry.label,
        template_id: entry.template_id,
        dwell_s: entry.dwell_s ?? defaultDwell,
        tactical_mode_hint: entry.tactical_mode_hint,
        meta,
      }),
    );
    runMeta.set(entry.entry_id, meta);
  }

  const batch = experimentBatchSpecSchema.parse({
    schema: "rt_experiment_batch_v1",
    experiment_id: spec.experiment_id,
    default_dwell_s: defaultDwell,
    runs,
  });
  return { batch, runMeta };
}

export function cartesianProductSize(
  axes: NonNullable<ExperimentSpec["matrix_axes"]>,
): number {
  return axes.reduce((acc, axis) => acc * axis.values.length, 1);
}

function cartesianProduct(
  axes: NonNullable<ExperimentSpec["matrix_axes"]>,
): Record<string, string>[] {
  let combos: Record<string, string>[] = [{}];
  for (const axis of axes) {
    const next: Record<string, string>[] = [];
    for (const combo of combos) {
      for (const value of axis.values) {
        next.push({ ...combo, [axis.axis_id]: String(value) });
      }
    }
    combos = next;
  }
  return combos;
}

function compileCartesian(spec: ExperimentSpec, fingerprint: string): CompileResult {
  const axes = spec.matrix_axes ?? [];
  const defaultDwell = spec.default_dwell_s ?? 2;
  const combos = cartesianProduct(axes);
  const runMeta = new Map<string, F5RunMeta>();
  const runs: ExperimentBatchSpec["runs"] = [];

  for (const coords of combos) {
    let templateId: string | undefined;
    let dwell = defaultDwell;
    let tacticalHint: string | undefined;
    let terrainRef: string | undefined;
    let f4Preset: string | undefined;

    for (const [axisId, value] of Object.entries(coords)) {
      if (axisId === "template_id") templateId = value;
      if (axisId === "dwell_s") dwell = Number(value);
      if (axisId === "tactical_mode_hint") tacticalHint = value;
      if (axisId === "terrain_profile_ref") terrainRef = value;
      if (axisId === "f4_layer_preset") f4Preset = value;
    }
    if (!templateId) {
      throw new Error("cartesian product missing template_id axis or value");
    }
    validateTemplateId(templateId);

    const runId = slugifyRunId(Object.values(coords));
    const meta: F5RunMeta = {
      experiment_class: spec.experiment_class,
      spec_fingerprint: fingerprint,
      matrix_coords: coords,
    };
    if (terrainRef) meta.terrain_profile_ref = terrainRef;
    if (f4Preset) meta.f4_layer_preset = f4Preset;

    runs.push(
      buildBatchRun(fingerprint, spec.experiment_class, {
        run_id: runId,
        label: axisSignature(coords),
        template_id: templateId,
        dwell_s: dwell,
        tactical_mode_hint: tacticalHint,
        meta,
      }),
    );
    runMeta.set(runId, meta);
  }

  const batch = experimentBatchSpecSchema.parse({
    schema: "rt_experiment_batch_v1",
    experiment_id: spec.experiment_id,
    default_dwell_s: defaultDwell,
    runs,
  });
  return { batch, runMeta };
}

function compileRepeatExpand(spec: ExperimentSpec, fingerprint: string): CompileResult {
  const rc = spec.repeat_config;
  if (!rc) throw new Error("repeat_config required");
  const defaultDwell = spec.default_dwell_s ?? rc.base_entry.dwell_s ?? 2;
  const baseId = rc.base_entry.template_id.replace(/_v1$/, "").slice(0, 20) || "repeat";
  const runMeta = new Map<string, F5RunMeta>();
  const runs: ExperimentBatchSpec["runs"] = [];

  for (let index = 0; index < rc.count; index += 1) {
    const runId = `${baseId}-r${index}`;
    const meta: F5RunMeta = {
      experiment_class: spec.experiment_class,
      spec_fingerprint: fingerprint,
      repeat_index: index,
      repeat_group_id: rc.repeat_group_id,
    };
    runs.push(
      buildBatchRun(fingerprint, spec.experiment_class, {
        run_id: runId,
        label: `${rc.base_entry.label} #${index}`,
        template_id: rc.base_entry.template_id,
        dwell_s: rc.base_entry.dwell_s ?? defaultDwell,
        meta,
      }),
    );
    runMeta.set(runId, meta);
  }

  const batch = experimentBatchSpecSchema.parse({
    schema: "rt_experiment_batch_v1",
    experiment_id: spec.experiment_id,
    default_dwell_s: defaultDwell,
    runs,
  });
  return { batch, runMeta };
}

export function compileExperimentSpec(spec: ExperimentSpec): CompileResult {
  assertCompileableSpec(spec);
  const fingerprint = computeSpecFingerprint(spec);

  switch (spec.compile_strategy) {
    case "explicit_list":
      return compileExplicitList(spec, fingerprint);
    case "cartesian":
      return compileCartesian(spec, fingerprint);
    case "repeat_expand":
      return compileRepeatExpand(spec, fingerprint);
    default:
      throw new Error(`unsupported compile_strategy: ${spec.compile_strategy}`);
  }
}

export function compileSpecToYaml(spec: ExperimentSpec): string {
  const { batch } = compileExperimentSpec(spec);
  return batchSpecToYaml(batch);
}

export { batchSpecToYaml };
