import {
  orchestrationFederationRecoveryContinuitySchema,
  replayFederationContinuityIndexSchema,
  replayFederationIndexSchema,
  replayFederationIntegrityReportSchema,
  replayFederationLineageGraphSchema,
  replayFederationManifestSchema,
  replayFederationPublicationCollectionSchema,
  type OrchestrationFederationRecoveryContinuity,
  type ReplayFederationContinuityIndex,
  type ReplayFederationIndex,
  type ReplayFederationIntegrityReport,
  type ReplayFederationLineageGraph,
  type ReplayFederationManifest,
  type ReplayFederationPublicationCollection,
} from "./federationSchema";

const BASE = "/demo/federation";

export async function loadFederationManifest(): Promise<ReplayFederationManifest> {
  const res = await fetch(`${BASE}/replay_federation_manifest_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation manifest: ${res.status}`);
  return replayFederationManifestSchema.parse(await res.json());
}

export async function loadFederationIndex(): Promise<ReplayFederationIndex> {
  const res = await fetch(`${BASE}/replay_federation_index_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation index: ${res.status}`);
  return replayFederationIndexSchema.parse(await res.json());
}

export async function loadFederationLineageGraph(): Promise<ReplayFederationLineageGraph> {
  const res = await fetch(`${BASE}/replay_federation_lineage_graph_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation lineage graph: ${res.status}`);
  return replayFederationLineageGraphSchema.parse(await res.json());
}

export async function loadFederationIntegrityReport(): Promise<ReplayFederationIntegrityReport> {
  const res = await fetch(`${BASE}/audits/replay_federation_integrity_report_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation integrity report: ${res.status}`);
  return replayFederationIntegrityReportSchema.parse(await res.json());
}

export async function loadFederationPublicationCollection(): Promise<ReplayFederationPublicationCollection> {
  const res = await fetch(`${BASE}/replay_federation_publication_collection_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation publication collection: ${res.status}`);
  return replayFederationPublicationCollectionSchema.parse(await res.json());
}

export async function loadFederationContinuityIndex(): Promise<ReplayFederationContinuityIndex> {
  const res = await fetch(`${BASE}/replay_federation_continuity_index_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation continuity index: ${res.status}`);
  return replayFederationContinuityIndexSchema.parse(await res.json());
}

export async function loadFederationRecoveryContinuity(): Promise<OrchestrationFederationRecoveryContinuity> {
  const res = await fetch(`${BASE}/audits/orchestration_federation_recovery_continuity_v1.json`);
  if (!res.ok) throw new Error(`Failed to load federation recovery continuity: ${res.status}`);
  return orchestrationFederationRecoveryContinuitySchema.parse(await res.json());
}
