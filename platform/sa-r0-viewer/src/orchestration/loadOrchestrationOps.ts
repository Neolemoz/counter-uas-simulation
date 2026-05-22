import {
  asyncBatchAuditSchema,
  asyncIntegrityReportSchema,
  orchestrationAsyncManifestSchema,
  orchestrationIntegrityReportSchema,
  orchestrationOpsManifestSchema,
  orchestrationRecoveryReportSchema,
  queueClaimTokenSchema,
  reconciliationLineageIndexSchema,
  workerExecutionRecordSchema,
  type AsyncBatchAudit,
  type AsyncIntegrityReport,
  type OrchestrationAsyncManifest,
  type OrchestrationIntegrityReport,
  type OrchestrationOpsManifest,
  type OrchestrationRecoveryReport,
  type QueueClaimToken,
  type ReconciliationLineageIndex,
  type WorkerExecutionRecord,
} from "./orchestrationOpsSchema";

export async function loadOrchestrationOpsManifest(
  manifestId: string,
): Promise<OrchestrationOpsManifest | null> {
  const url = `/demo/orchestration/ops/${manifestId}_ops.json`;
  const res = await fetch(url);
  if (!res.ok) return null;
  return orchestrationOpsManifestSchema.parse(await res.json());
}

export async function loadOrchestrationIntegrityReport(): Promise<OrchestrationIntegrityReport | null> {
  const res = await fetch("/demo/orchestration/integrity_report.json");
  if (!res.ok) return null;
  return orchestrationIntegrityReportSchema.parse(await res.json());
}

export async function loadOrchestrationAsyncManifest(
  manifestId: string,
): Promise<OrchestrationAsyncManifest | null> {
  const url = `/demo/orchestration/async/${manifestId}_async.json`;
  const res = await fetch(url);
  if (!res.ok) return null;
  return orchestrationAsyncManifestSchema.parse(await res.json());
}

export async function loadAsyncIntegrityReport(): Promise<AsyncIntegrityReport | null> {
  const res = await fetch("/demo/orchestration/async_integrity_report.json");
  if (!res.ok) return null;
  return asyncIntegrityReportSchema.parse(await res.json());
}

export async function loadWorkerExecutionRecord(
  ref: string,
): Promise<WorkerExecutionRecord | null> {
  const name = ref.split("/").pop();
  if (!name) return null;
  const res = await fetch(`/demo/orchestration/workers/${name}`);
  if (!res.ok) return null;
  return workerExecutionRecordSchema.parse(await res.json());
}

export async function loadQueueClaimToken(ref: string): Promise<QueueClaimToken | null> {
  const name = ref.split("/").pop();
  if (!name) return null;
  const res = await fetch(`/demo/orchestration/claims/${name}`);
  if (!res.ok) return null;
  return queueClaimTokenSchema.parse(await res.json());
}

export async function loadRecoveryReport(
  manifestId: string,
): Promise<OrchestrationRecoveryReport | null> {
  const url = `/demo/orchestration/recovery/${manifestId}_recovery_report.json`;
  const res = await fetch(url);
  if (!res.ok) return null;
  return orchestrationRecoveryReportSchema.parse(await res.json());
}

export async function loadAsyncBatchAudit(): Promise<AsyncBatchAudit | null> {
  const res = await fetch("/demo/orchestration/synthesis/async_batch_audit_v1.json");
  if (!res.ok) return null;
  return asyncBatchAuditSchema.parse(await res.json());
}

export async function loadReconciliationLineageIndex(): Promise<ReconciliationLineageIndex | null> {
  const res = await fetch("/demo/orchestration/reconciliation/reconciliation_lineage_index_v1.json");
  if (!res.ok) return null;
  return reconciliationLineageIndexSchema.parse(await res.json());
}

export function readOrchestrationManifestFromUrl(): string | null {
  if (typeof window === "undefined") return null;
  return new URLSearchParams(window.location.search).get("orchestration_manifest");
}
