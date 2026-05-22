import { z } from "zod";

export const OPS_LADDER_STATUSES = [
  "pending",
  "validated",
  "queued",
  "executed",
  "replay_generated",
  "archived",
] as const;

export const orchestrationOpsManifestSchema = z.object({
  artifact_type: z.literal("experiment_orchestration_ops_manifest_v1"),
  schema_version: z.literal("1"),
  manifest_id: z.string(),
  operations_status: z.enum(OPS_LADDER_STATUSES),
  scenario_pack_ids: z.array(z.string()).optional(),
  manifest_fingerprint: z.string().optional(),
  queue_snapshot_ref: z.string().optional(),
  audit_report_ref: z.string().optional(),
  operations_lineage: z
    .array(
      z.object({
        event_id: z.string(),
        from_status: z.string(),
        to_status: z.string(),
        actor: z.string().optional(),
        notes: z.string().optional(),
      }),
    )
    .optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
  updated_at: z.string().optional(),
});

export type OrchestrationOpsManifest = z.infer<typeof orchestrationOpsManifestSchema>;

export const orchestrationIntegrityReportSchema = z.object({
  artifact_type: z.literal("orchestration_integrity_report_v1"),
  schema_version: z.string(),
  checked_at: z.string().optional(),
  ok: z.boolean(),
  pack_count: z.number().optional(),
  manifest_count: z.number().optional(),
  errors: z.array(z.string()).optional(),
  warnings: z.array(z.string()).optional(),
  per_manifest: z
    .record(
      z.object({
        manifest_id: z.string().optional(),
        pack_ids: z.array(z.string()).optional(),
        errors: z.array(z.string()).optional(),
        warnings: z.array(z.string()).optional(),
      }),
    )
    .optional(),
  per_pack: z
    .record(
      z.object({
        pack_id: z.string().optional(),
        errors: z.array(z.string()).optional(),
        warnings: z.array(z.string()).optional(),
      }),
    )
    .optional(),
  governance_banner: z.string().optional(),
});

export type OrchestrationIntegrityReport = z.infer<typeof orchestrationIntegrityReportSchema>;

export const ASYNC_EXECUTION_STATUSES = [
  "retrying",
  "failed",
  "quarantined",
  "superseded",
] as const;

export const orchestrationAsyncManifestSchema = z.object({
  artifact_type: z.literal("experiment_orchestration_async_manifest_v1"),
  schema_version: z.literal("1"),
  manifest_id: z.string(),
  async_execution_status: z.enum(ASYNC_EXECUTION_STATUSES).optional(),
  execution_fingerprint: z.string().optional(),
  replay_fingerprint: z.string().optional(),
  queue_snapshot_hash: z.string().optional(),
  queue_claim_ref: z.string().optional(),
  worker_execution_refs: z.array(z.string()).optional(),
  async_lineage: z
    .array(
      z.object({
        event_id: z.string(),
        from_status: z.string().nullable().optional(),
        to_status: z.string(),
        actor: z.string().optional(),
        notes: z.string().optional(),
      }),
    )
    .optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
  updated_at: z.string().optional(),
});

export type OrchestrationAsyncManifest = z.infer<typeof orchestrationAsyncManifestSchema>;

export const queueClaimTokenSchema = z.object({
  artifact_type: z.literal("queue_claim_token_v1"),
  schema_version: z.literal("1"),
  claim_id: z.string(),
  manifest_id: z.string(),
  queue_id: z.string().optional(),
  claim_status: z.enum(["open", "closed"]).optional(),
  snapshot_hash: z.string().optional(),
  queue_snapshot_ref: z.string().optional(),
  terminal_reason: z.string().optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
});

export type QueueClaimToken = z.infer<typeof queueClaimTokenSchema>;

export const orchestrationRecoveryReportSchema = z.object({
  artifact_type: z.literal("orchestration_recovery_report_v1"),
  schema_version: z.literal("1"),
  manifest_id: z.string(),
  generated_at: z.string().optional(),
  async_execution_status: z.enum(ASYNC_EXECUTION_STATUSES).optional(),
  retry_chain: z.array(z.record(z.unknown())).optional(),
  recovery_issues: z
    .array(z.object({ kind: z.string(), message: z.string(), manifest_id: z.string().optional() }))
    .optional(),
  recovery_warnings: z
    .array(z.object({ kind: z.string(), message: z.string(), manifest_id: z.string().optional() }))
    .optional(),
  replay_reconciliation: z.record(z.unknown()).optional(),
  recovery_continuity_ok: z.boolean().optional(),
  superseded: z.boolean().optional(),
  quarantine_hold: z.boolean().optional(),
  governance_banner: z.string().optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
});

export type OrchestrationRecoveryReport = z.infer<typeof orchestrationRecoveryReportSchema>;

export const asyncBatchAuditSchema = z.object({
  artifact_type: z.literal("orchestration_async_batch_audit_v1"),
  schema_version: z.literal("1"),
  generated_at: z.string().optional(),
  ok: z.boolean(),
  async_manifest_count: z.number().optional(),
  status_counts: z.record(z.number()).optional(),
  recovery_issue_count: z.number().optional(),
  recovery_warning_count: z.number().optional(),
  failed_manifest_ids: z.array(z.string()).optional(),
  quarantined_manifest_ids: z.array(z.string()).optional(),
  governance_banner: z.string().optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
});

export type AsyncBatchAudit = z.infer<typeof asyncBatchAuditSchema>;

export const reconciliationLineageIndexSchema = z.object({
  artifact_type: z.literal("orchestration_reconciliation_lineage_index_v1"),
  schema_version: z.literal("1"),
  generated_at: z.string().optional(),
  retry_groups: z
    .array(
      z.object({
        manifest_id: z.string(),
        worker_refs: z.array(z.string()),
        attempt_count: z.number(),
      }),
    )
    .optional(),
  supersede_edges: z
    .array(
      z.object({
        from_manifest_id: z.string(),
        to_manifest_id: z.string().optional(),
        notes: z.string().optional(),
      }),
    )
    .optional(),
  quarantine_holds: z
    .array(z.object({ manifest_id: z.string(), reason: z.string().optional() }))
    .optional(),
  failed_executions: z
    .array(
      z.object({
        manifest_id: z.string(),
        async_execution_status: z.string().optional(),
      }),
    )
    .optional(),
  governance_banner: z.string().optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
});

export type ReconciliationLineageIndex = z.infer<typeof reconciliationLineageIndexSchema>;

export const asyncIntegrityReportSchema = z.object({
  artifact_type: z.literal("orchestration_async_integrity_report_v1"),
  schema_version: z.string(),
  checked_at: z.string().optional(),
  ok: z.boolean(),
  per_manifest: z.record(z.unknown()).optional(),
  issues: z
    .array(
      z.object({
        kind: z.string(),
        manifest_id: z.string().optional(),
        message: z.string(),
      }),
    )
    .optional(),
  warnings: z
    .array(
      z.object({
        kind: z.string(),
        manifest_id: z.string().optional(),
        message: z.string(),
      }),
    )
    .optional(),
  governance_banner: z.string().optional(),
});

export type AsyncIntegrityReport = z.infer<typeof asyncIntegrityReportSchema>;

export const workerExecutionRecordSchema = z.object({
  artifact_type: z.literal("worker_execution_record_v1"),
  schema_version: z.literal("1"),
  worker_id: z.string(),
  execution_attempt: z.number(),
  manifest_id: z.string(),
  claim_id: z.string().optional(),
  execution_fingerprint: z.string().optional(),
  retry_lineage: z.array(z.record(z.unknown())).optional(),
  deterministic_metadata: z.record(z.unknown()).optional(),
  governance: z
    .object({
      notice: z.string(),
      anti_claims: z.array(z.string()),
    })
    .optional(),
});

export type WorkerExecutionRecord = z.infer<typeof workerExecutionRecordSchema>;
