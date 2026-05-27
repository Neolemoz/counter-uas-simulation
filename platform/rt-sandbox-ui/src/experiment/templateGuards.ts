import { BUILTIN_TEMPLATE_ID_SET } from "./templateIds";

const BLOCKED_FRAGMENTS = [
  "fixtures/scenarios",
  "platform/sa-r0-viewer",
  "fixtures/orchestration",
  "fixtures/sa_r0",
  "replay_federation",
  "federation_index",
  "scenario_pack_ref",
] as const;

export const ALLOWED_MATRIX_AXIS_IDS = [
  "template_id",
  "tactical_mode_hint",
  "dwell_s",
  "terrain_profile_ref",
  "f4_layer_preset",
] as const;

export type AllowedMatrixAxisId = (typeof ALLOWED_MATRIX_AXIS_IDS)[number];

export function assertTemplateRefBlocked(ref: string): void {
  const low = ref.toLowerCase().replace(/\\/g, "/");
  for (const frag of BLOCKED_FRAGMENTS) {
    if (low.includes(frag)) {
      throw new Error(`RT template ref blocked: ${ref}`);
    }
  }
  if (ref.startsWith("/") || ref.startsWith("..")) {
    throw new Error(`RT template ref blocked: ${ref}`);
  }
}

export function assertBuiltinTemplateId(templateId: string): void {
  assertTemplateRefBlocked(templateId);
  if (!BUILTIN_TEMPLATE_ID_SET.has(templateId)) {
    throw new Error(`unknown template_id: ${templateId}`);
  }
}

export function assertAllowedMatrixAxis(axisId: string): void {
  if (!(ALLOWED_MATRIX_AXIS_IDS as readonly string[]).includes(axisId)) {
    throw new Error(`forbidden matrix axis: ${axisId}`);
  }
}
