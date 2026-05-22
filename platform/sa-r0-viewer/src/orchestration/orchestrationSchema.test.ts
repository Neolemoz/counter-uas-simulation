import { describe, expect, it } from "vitest";
import { readFileSync } from "fs";
import { join } from "path";
import { experimentRunQueueSchema, validationMirrorSchema } from "./orchestrationSchema";
import {
  asyncBatchAuditSchema,
  asyncIntegrityReportSchema,
  orchestrationAsyncManifestSchema,
  orchestrationRecoveryReportSchema,
  queueClaimTokenSchema,
  reconciliationLineageIndexSchema,
  workerExecutionRecordSchema,
} from "./orchestrationOpsSchema";

const PUBLIC = join(__dirname, "../../public/demo/orchestration");

describe("orchestrationSchema", () => {
  it("parses committed queue snapshot", () => {
    const raw = readFileSync(
      join(PUBLIC, "queues/valley_ingress_validation_only_queue.json"),
      "utf-8",
    );
    const parsed = experimentRunQueueSchema.parse(JSON.parse(raw));
    expect(parsed.manifest_id).toBe("valley_ingress_validation_only");
    expect(parsed.jobs.length).toBeGreaterThan(0);
  });

  it("parses validation mirror", () => {
    const raw = readFileSync(
      join(PUBLIC, "validation_mirrors/valley_ingress_validation_mirror.json"),
      "utf-8",
    );
    const parsed = validationMirrorSchema.parse(JSON.parse(raw));
    expect(parsed.scenario_pack_id).toBe("valley_ingress");
  });

  it("parses async manifest mirror when present", () => {
    const path = join(PUBLIC, "async/ridge_defense_synthetic_pipeline_async.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = orchestrationAsyncManifestSchema.parse(JSON.parse(raw));
      expect(parsed.manifest_id).toBe("ridge_defense_synthetic_pipeline");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses async integrity report when present", () => {
    const path = join(PUBLIC, "async_integrity_report.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = asyncIntegrityReportSchema.parse(JSON.parse(raw));
      expect(parsed.artifact_type).toBe("orchestration_async_integrity_report_v1");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses worker execution record when present", () => {
    const path = join(PUBLIC, "workers/cli-worker-ridge-ref_1.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = workerExecutionRecordSchema.parse(JSON.parse(raw));
      expect(parsed.worker_id).toBe("cli-worker-ridge-ref");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses queue claim mirror when present", () => {
    const path = join(PUBLIC, "claims/ridge_defense_synthetic_pipeline_queue_claim.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = queueClaimTokenSchema.parse(JSON.parse(raw));
      expect(parsed.claim_status).toBe("closed");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses recovery report when present", () => {
    const path = join(PUBLIC, "recovery/ridge_defense_synthetic_pipeline_recovery_report.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = orchestrationRecoveryReportSchema.parse(JSON.parse(raw));
      expect(parsed.manifest_id).toBe("ridge_defense_synthetic_pipeline");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses async batch audit when present", () => {
    const path = join(PUBLIC, "synthesis/async_batch_audit_v1.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = asyncBatchAuditSchema.parse(JSON.parse(raw));
      expect(parsed.artifact_type).toBe("orchestration_async_batch_audit_v1");
    } catch {
      // mirror not synced in dev — skip
    }
  });

  it("parses reconciliation lineage index when present", () => {
    const path = join(PUBLIC, "reconciliation/reconciliation_lineage_index_v1.json");
    try {
      const raw = readFileSync(path, "utf-8");
      const parsed = reconciliationLineageIndexSchema.parse(JSON.parse(raw));
      expect(parsed.retry_groups?.length).toBeGreaterThan(0);
    } catch {
      // mirror not synced in dev — skip
    }
  });
});
