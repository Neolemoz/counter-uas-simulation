import {
  corpusReleaseManifestSchema,
  replayCorpusDriftReportSchema,
  type CorpusReleaseManifest,
  type ReplayCorpusDriftReport,
} from "../synthesis/synthesisSchema";

export const DRIFT_REPORT_URL = "/demo/corpus_audits/replay_corpus_drift_report_v1.json";
export const DEFAULT_RELEASE_ID = "sa_r0_corpus_r1_r1";

export async function loadCorpusDriftReport(): Promise<ReplayCorpusDriftReport> {
  const res = await fetch(DRIFT_REPORT_URL);
  if (!res.ok) throw new Error(`Failed to load corpus drift report: ${res.status}`);
  const data: unknown = await res.json();
  return replayCorpusDriftReportSchema.parse(data);
}

export async function loadCorpusReleaseManifest(
  releaseId: string = DEFAULT_RELEASE_ID,
): Promise<CorpusReleaseManifest> {
  const res = await fetch(
    `/demo/corpus_releases/${releaseId}/replay_corpus_release_manifest_v1.json`,
  );
  if (!res.ok) throw new Error(`Failed to load release manifest: ${res.status}`);
  const data: unknown = await res.json();
  return corpusReleaseManifestSchema.parse(data);
}
