import {
  assertNoForbiddenManifestRefKeys,
  COHORT_GOVERNANCE_BANNER,
  experimentCohortIndexSchema,
  type ExperimentCohortIndex,
} from "./cohortSchema";
import type { ParseResult } from "./experimentImportGuards";
import { formatImportError } from "./experimentImportGuards";

const BLOCKED_REF_FRAGMENTS = [
  "fixtures/scenarios",
  "platform/sa-r0-viewer",
  "fixtures/orchestration",
  "fixtures/sa_r0",
  "replay_sa",
  "replay_federation",
  "federation_index",
  "scenario_pack_ref",
] as const;

const COHORT_BANNER_SUBSTRING = "RT EXPERIMENT COHORT";

function zodMessage(err: unknown): string {
  if (err && typeof err === "object" && "message" in err) {
    return String((err as { message: string }).message);
  }
  return "validation failed";
}

export function assertCohortManifestRefAllowed(ref: string): void {
  const low = ref.toLowerCase().replace(/\\/g, "/");
  if (ref.startsWith("/") || ref.startsWith("..")) {
    throw new Error(`RT cohort manifest_ref blocked: ${ref}`);
  }
  for (const frag of BLOCKED_REF_FRAGMENTS) {
    if (low.includes(frag)) {
      throw new Error(`RT cohort manifest_ref blocked: ${ref}`);
    }
  }
}

export function assertCohortBanner(banner: string): void {
  if (banner === COHORT_GOVERNANCE_BANNER) return;
  if (!banner.includes(COHORT_BANNER_SUBSTRING)) {
    throw new Error("cohort governance_banner must include RT EXPERIMENT COHORT normative line");
  }
}

export function validateCohortIndex(index: ExperimentCohortIndex): ExperimentCohortIndex {
  assertCohortBanner(index.governance_banner);
  for (const ref of index.manifest_refs) {
    assertCohortManifestRefAllowed(ref.manifest_ref);
  }
  return index;
}

export function safeParseCohortIndex(text: string): ParseResult<ExperimentCohortIndex> {
  try {
    const raw = JSON.parse(text);
    assertNoForbiddenManifestRefKeys(raw);
    const parsed = experimentCohortIndexSchema.parse(raw);
    validateCohortIndex(parsed);
    return { ok: true, data: parsed };
  } catch (err) {
    return { ok: false, error: formatImportError(zodMessage(err)) };
  }
}
